//! Native codec for **Luwu Dynamics XGO** quadrupeds — target: **XGO-Lite 2**
//! (Raspberry Pi **CM4** board).
//!
//! Hardware: desktop quadruped, 12 hobby servos (4 legs × 3 joints: knee,
//! thigh, hip), onboard IMU, STM32 motion controller wired to the CM4 primary
//! UART: **115200 baud, 8N1, no flow control** (`/dev/ttyAMA0` on the robot's
//! Linux).
//!
//! Protocol (register-oriented, verified against the official `xgolib` 1.4.x
//! library of Luwu Dynamics):
//!
//! ```text
//! 0x55 0x00 LEN MODE ADDR data… CHK 0x00 0xAA
//! ```
//!
//! - `LEN`  = **total frame length** in bytes (`data.len() + 8`, minimum 9);
//! - `MODE` = `0x00` write, `0x02` read request;
//! - `CHK`  = `255 - ((LEN + MODE + ADDR + Σ data) % 256)`;
//! - a read request carries the requested byte count as its single `data`
//!   byte; responses echo the register address with the payload.
//!
//! The STM32 **never streams telemetry**: registers must be polled. The
//! adapter therefore implements `poll_request` with a rotating schedule
//! (joint angles → roll → pitch → yaw → battery) at 50 Hz and keeps a
//! persistent snapshot, so every answered poll refreshes the `StateVector`
//! slots it owns: `[0..12)` joint angles (rad), `[12]` roll, `[13]` pitch,
//! `[14]` yaw (rad), `[15]` battery (V; register unit 0.1 V).
//!
//! `CommandVector` mapping:
//! - slots `[0..12)` → MOTOR_ANGLE registers `0x50..0x5B` in a single
//!   multi-register write (rad → deg → u8 through the official `conver2u8`
//!   transfer function with the XGO-Lite mechanical limits);
//! - slot `[12]` gait speed Vx (m/s), `[13]` Vy (m/s), `[14]` yaw rate
//!   (rad/s) → velocity registers `0x30/0x31/0x32` (limits 25/18 cm/s,
//!   100 °/s);
//! - slot `[15]` → ACTION register `0x3E` (e.g. 1 = stand up, 255 = reset).
//!
//! ESTOP maps to XGO ACTION 255: the dog stops every motion and restores its
//! initial parameters. Commanding "all joints to zero" would instead fold the
//! legs into an unsafe mechanical pose.

use crate::io_bridge::adapters::{BrandAdapter, FrameStrategy};
use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::robot_brand::RobotBrand;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Mutex;

const FRAME_HEADER0: u8 = 0x55;
const FRAME_HEADER1: u8 = 0x00;
const MODE_WRITE: u8 = 0x00;
const MODE_READ: u8 = 0x02;
const FRAME_TAIL0: u8 = 0x00;
const FRAME_TAIL1: u8 = 0xAA;
/// Bytes around the payload: `0x55 0x00 LEN MODE ADDR` + `CHK 0x00 0xAA`.
const FRAME_OVERHEAD: usize = 8;
/// Smallest valid frame (single data byte, e.g. a read request).
const FRAME_MIN_LEN: usize = 9;
/// Sanity cap for the LEN byte while re-synchronizing the stream.
const FRAME_MAX_LEN: usize = 64;

// XGO register map (official xgolib register addresses).
const REG_BATTERY: u8 = 0x01;
/// `0x30` VX, `0x31` VY, `0x32` VYAW (three consecutive registers).
const REG_VX: u8 = 0x30;
/// ACTION macro register (255 = stop every motion and reset).
const REG_ACTION: u8 = 0x3E;
/// MOTOR_ANGLE base: `0x50..0x5B` = the 12 leg joints (0x5C+ = claw/arm).
const REG_MOTOR_ANGLE: u8 = 0x50;
const REG_ROLL: u8 = 0x62;
const REG_PITCH: u8 = 0x63;
const REG_YAW: u8 = 0x64;

/// Number of leg motors: 4 legs × (knee, thigh, hip).
pub const XGO_LEG_MOTORS: usize = 12;

/// XGO-Lite mechanical joint ranges in **degrees**, official `MOTOR_LIMIT`
/// of xgolib, indexed by `joint_index % 3`:
/// `[knee (-70..50), thigh (-70..90), hip (-30..30)]`.
pub const XGO_LITE_JOINT_LIMITS_DEG: [(f32, f32); 3] = [
    (-70.0, 50.0),
    (-70.0, 90.0),
    (-30.0, 30.0),
];

/// Gait velocity limits (XGO-Lite): Vx, Vy in cm/s and yaw rate in °/s.
const VX_LIMIT_CMS: f32 = 25.0;
const VY_LIMIT_CMS: f32 = 18.0;
const VYAW_LIMIT_DEGS: f32 = 100.0;

/// Telemetry snapshot length: 12 joints + roll/pitch/yaw + battery.
const SNAPSHOT_LEN: usize = XGO_LEG_MOTORS + 4;

/// Poll schedule: joint angles, roll, pitch, yaw, battery — then repeat.
const POLL_PHASES: u8 = 5;
/// Poll period: 20 ms → every channel refreshed at 10 Hz (~2.3 kB/s bus).
const POLL_PERIOD_MS: u64 = 20;

/// Native codec of the Luwu Dynamics XGO register bus (XGO-Lite 2, CM4).
///
/// `Send + Sync`: the poll phase is atomic and the telemetry snapshot is
/// mutex-protected exactly like the sanitizer/jerk-limiter of the HAL loop
/// (the lock is never held across an `.await`).
pub struct XgoUart {
    /// Rotating poll schedule state.
    poll_phase: AtomicU8,
    /// Last known value of every owned `StateVector` slot: each answered
    /// register read republishes a complete vector.
    snapshot: Mutex<[f32; MAX_JOINTS]>,
}

impl Default for XgoUart {
    fn default() -> Self {
        Self::new()
    }
}

impl XgoUart {
    pub fn new() -> Self {
        Self {
            poll_phase: AtomicU8::new(0),
            snapshot: Mutex::new([0.0; MAX_JOINTS]),
        }
    }
}

/// Official `conver2u8` transfer function: physical value → 0..255 byte
/// across the `[min, max]` range, clamped at both ends (xgolib-compatible).
fn conver2u8(value: f32, min: f32, max: f32) -> u8 {
    if value >= max {
        0xFF
    } else if value <= min {
        0
    } else {
        ((255.0 / (max - min)) * (value - min)) as u8
    }
}

/// Inverse `conver2float`: 0..255 byte → physical value across `[min, max]`.
fn conver2float(raw: u8, min: f32, max: f32) -> f32 {
    raw as f32 / 255.0 * (max - min) + min
}

fn deg2rad(deg: f32) -> f32 {
    deg * std::f32::consts::PI / 180.0
}

fn rad2deg(rad: f32) -> f32 {
    rad * 180.0 / std::f32::consts::PI
}

/// XGO frame checksum: `255 - ((LEN + MODE + ADDR + Σ data) % 256)`.
fn checksum(len: u8, mode: u8, addr: u8, data: &[u8]) -> u8 {
    let mut sum = len as u16 + mode as u16 + addr as u16;
    for &byte in data {
        sum += byte as u16;
    }
    (255 - (sum % 256)) as u8
}

/// Appends one XGO frame at the front of `out` (the caller slices the free
/// tail). Returns the bytes written, 0 when `out` has not enough room.
fn push_frame(out: &mut [u8], mode: u8, addr: u8, data: &[u8]) -> usize {
    let total = data.len() + FRAME_OVERHEAD;
    if out.len() < total || total > u8::MAX as usize {
        return 0;
    }
    let len = total as u8;
    out[0] = FRAME_HEADER0;
    out[1] = FRAME_HEADER1;
    out[2] = len;
    out[3] = mode;
    out[4] = addr;
    out[5..5 + data.len()].copy_from_slice(data);
    out[5 + data.len()] = checksum(len, mode, addr, data);
    out[6 + data.len()] = FRAME_TAIL0;
    out[7 + data.len()] = FRAME_TAIL1;
    total
}
/// Result of one scan of the receive accumulator.
enum Scan {
    /// Complete, checksum-valid frame: (total_len, addr, data_len).
    Frame {
        len: usize,
        addr: u8,
        data_len: usize,
    },
    /// Header found but the frame is not buffered yet.
    Incomplete,
    /// Bytes to flush (no header / corrupt frame) before re-scanning.
    Garbage { skip: usize },
}

/// Scans `acc` for ONE complete XGO frame. The frame must sit at the very
/// front: anything before its header is reported as [`Scan::Garbage`] so the
/// caller can flush it and re-synchronize on the next call.
fn scan_frame(acc: &[u8]) -> Scan {
    let header = acc[..acc.len().saturating_sub(1)]
        .windows(2)
        .position(|w| w == [FRAME_HEADER0, FRAME_HEADER1]);
    let Some(start) = header else {
        // No header pair: flush everything except a possible trailing 0x55.
        let keep = acc
            .iter()
            .rposition(|&b| b == FRAME_HEADER0)
            .map(|p| p + 1)
            .unwrap_or(acc.len());
        return Scan::Garbage { skip: keep };
    };
    if start > 0 {
        return Scan::Garbage { skip: start };
    }
    if acc.len() < 3 {
        return Scan::Incomplete;
    }
    let len = acc[2] as usize;
    if !(FRAME_MIN_LEN..=FRAME_MAX_LEN).contains(&len) {
        // Corrupt length: drop the fake header pair and re-synchronize.
        return Scan::Garbage { skip: 2 };
    }
    if acc.len() < len {
        return Scan::Incomplete;
    }
    let mode = acc[3];
    let addr = acc[4];
    let data_len = len - FRAME_OVERHEAD;
    let chk = acc[5 + data_len];
    if acc[len - 2] != FRAME_TAIL0 || acc[len - 1] != FRAME_TAIL1 {
        return Scan::Garbage { skip: 2 };
    }
    if checksum(acc[2], mode, addr, &acc[5..5 + data_len]) != chk {
        return Scan::Garbage { skip: 2 };
    }
    Scan::Frame {
        len,
        addr,
        data_len,
    }
}
impl BrandAdapter for XgoUart {
    fn brand(&self) -> RobotBrand {
        RobotBrand::LuwuDynamics
    }

    fn protocol_label(&self) -> &'static str {
        "XGO Register UART 0x55"
    }

    fn frame_strategy(&self) -> FrameStrategy {
        FrameStrategy::RegisterFramed {
            header: FRAME_HEADER0,
        }
    }

    fn buffer_hint(&self) -> usize {
        256
    }

    /// Decodes ONE answered register read and merges it into the persistent
    /// snapshot. Returns `n == 0` while flushing garbage so the HAL loop can
    /// re-synchronize without publishing fake telemetry.
    fn decode_telemetry(
        &self,
        acc: &[u8],
        joint_count: usize,
    ) -> Option<(usize, [f32; MAX_JOINTS], usize)> {
        match scan_frame(acc) {
            Scan::Frame {
                len,
                addr,
                data_len,
            } => {
                let data = &acc[5..5 + data_len];
                let mut guard = self.snapshot.lock().expect("XGO snapshot poisoned");
                let n_joints = joint_count.min(XGO_LEG_MOTORS);
                let motor_base = addr.wrapping_sub(REG_MOTOR_ANGLE) as usize;
                if motor_base < XGO_LEG_MOTORS {
                    // Motor-angle read: payload is consecutive u8 angles.
                    for (i, &raw) in data.iter().enumerate() {
                        let joint = motor_base + i;
                        if joint >= n_joints {
                            break;
                        }
                        let (min, max) = XGO_LITE_JOINT_LIMITS_DEG[joint % 3];
                        guard[joint] = deg2rad(conver2float(raw, min, max));
                    }
                } else if (addr == REG_ROLL || addr == REG_PITCH || addr == REG_YAW)
                    && data.len() >= 4
                {
                    // IMU attitude: f32 on the wire is little-endian, degrees.
                    let raw = f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                    if raw.is_finite() {
                        let slot = match addr {
                            REG_ROLL => XGO_LEG_MOTORS,
                            REG_PITCH => XGO_LEG_MOTORS + 1,
                            _ => XGO_LEG_MOTORS + 2,
                        };
                        guard[slot] = deg2rad(raw);
                    }
                } else if addr == REG_BATTERY && !data.is_empty() {
                    // Battery register: 0.1 V units (typical 60..84 = 6.0..8.4 V).
                    guard[XGO_LEG_MOTORS + 3] = data[0] as f32 * 0.1;
                }
                let values = *guard;
                drop(guard);
                Some((len, values, SNAPSHOT_LEN))
            }
            Scan::Garbage { skip } => {
                let values = *self.snapshot.lock().expect("XGO snapshot poisoned");
                Some((skip, values, 0))
            }
            Scan::Incomplete => None,
        }
    }
    /// Encodes the command into consecutive XGO register writes: joint
    /// positions (multi-register `0x50` write), then gait velocities
    /// (`0x30..0x32`) and the action macro (`0x3E`) when the slots are used.
    fn encode_command(&self, command: &CommandVector, out: &mut [u8]) -> usize {
        let mut written = 0;

        // 1) Joint position targets (rad) → MOTOR_ANGLE registers 0x50..0x5B.
        let n_joints = command.len.min(XGO_LEG_MOTORS);
        if n_joints > 0 {
            let mut payload = [0u8; XGO_LEG_MOTORS];
            for (i, item) in payload.iter_mut().enumerate().take(n_joints) {
                let (min, max) = XGO_LITE_JOINT_LIMITS_DEG[i % 3];
                *item = conver2u8(rad2deg(command.target_actuators[i]), min, max);
            }
            written += push_frame(
                &mut out[written..],
                MODE_WRITE,
                REG_MOTOR_ANGLE,
                &payload[..n_joints],
            );
        }

        // 2) Gait velocities: Vx (m/s), Vy (m/s), yaw rate (rad/s).
        if command.len >= XGO_LEG_MOTORS + 3 {
            let payload = [
                conver2u8(
                    command.target_actuators[XGO_LEG_MOTORS] * 100.0,
                    -VX_LIMIT_CMS,
                    VX_LIMIT_CMS,
                ),
                conver2u8(
                    command.target_actuators[XGO_LEG_MOTORS + 1] * 100.0,
                    -VY_LIMIT_CMS,
                    VY_LIMIT_CMS,
                ),
                conver2u8(
                    rad2deg(command.target_actuators[XGO_LEG_MOTORS + 2]),
                    -VYAW_LIMIT_DEGS,
                    VYAW_LIMIT_DEGS,
                ),
            ];
            written += push_frame(&mut out[written..], MODE_WRITE, REG_VX, &payload);
        }

        // 3) Action macro trigger (stand, prone, reset, …).
        if command.len >= XGO_LEG_MOTORS + 4 {
            let payload = [command.target_actuators[XGO_LEG_MOTORS + 3].clamp(0.0, 255.0) as u8];
            written += push_frame(&mut out[written..], MODE_WRITE, REG_ACTION, &payload);
        }

        written
    }

    /// XGO safe-stop: ACTION 255 stops every motion and restores the initial
    /// parameters. (Zeroed joint targets would command an unsafe crouch.)
    fn encode_estop(&self, _joint_count: usize, out: &mut [u8]) -> usize {
        push_frame(out, MODE_WRITE, REG_ACTION, &[255])
    }

    /// Rotating telemetry poll: joint angles → roll → pitch → yaw → battery.
    fn poll_request(&self, out: &mut [u8]) -> usize {
        let phase = self.poll_phase.fetch_add(1, Ordering::Relaxed) % POLL_PHASES;
        let (addr, read_len) = match phase {
            0 => (REG_MOTOR_ANGLE, XGO_LEG_MOTORS as u8),
            1 => (REG_ROLL, 4),
            2 => (REG_PITCH, 4),
            3 => (REG_YAW, 4),
            _ => (REG_BATTERY, 1),
        };
        push_frame(out, MODE_READ, addr, &[read_len])
    }

    fn poll_period_ms(&self) -> u64 {
        POLL_PERIOD_MS
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a response frame exactly like the STM32 firmware does.
    fn response_frame(mode: u8, addr: u8, data: &[u8]) -> Vec<u8> {
        let mut frame = Vec::with_capacity(data.len() + FRAME_OVERHEAD);
        frame.push(FRAME_HEADER0);
        frame.push(FRAME_HEADER1);
        frame.push((data.len() + FRAME_OVERHEAD) as u8);
        frame.push(mode);
        frame.push(addr);
        frame.extend_from_slice(data);
        frame.push(checksum(
            (data.len() + FRAME_OVERHEAD) as u8,
            mode,
            addr,
            data,
        ));
        frame.push(FRAME_TAIL0);
        frame.push(FRAME_TAIL1);
        frame
    }

    #[test]
    fn motor_command_frame_layout_and_checksum() {
        let adapter = XgoUart::new();
        let cmd = CommandVector::zeros(XGO_LEG_MOTORS); // 0.0 rad everywhere
        let mut out = [0u8; 256];
        let n = adapter.encode_command(&cmd, &mut out);
        assert_eq!(n, XGO_LEG_MOTORS + FRAME_OVERHEAD); // one 20-byte frame
        assert_eq!(out[0], 0x55);
        assert_eq!(out[1], 0x00);
        assert_eq!(out[2], 20); // LEN = total frame length
        assert_eq!(out[3], MODE_WRITE);
        assert_eq!(out[4], REG_MOTOR_ANGLE);
        // Official transfer function of 0.0° over the XGO-Lite ranges:
        // knee [-70,50] → 148, thigh [-70,90] → 111, hip [-30,30] → 127.
        assert_eq!(out[5], 148);
        assert_eq!(out[6], 111);
        assert_eq!(out[7], 127);
        assert_eq!(
            out[5 + XGO_LEG_MOTORS],
            checksum(
                20,
                MODE_WRITE,
                REG_MOTOR_ANGLE,
                &out[5..5 + XGO_LEG_MOTORS]
            )
        );
        assert_eq!(out[18], FRAME_TAIL0);
        assert_eq!(out[19], FRAME_TAIL1);
    }

    #[test]
    fn joint_targets_clamp_to_mechanical_range() {
        let adapter = XgoUart::new();
        // 120° thigh target exceeds the [-70, 90] range → 0xFF.
        let mut cmd = CommandVector::zeros(XGO_LEG_MOTORS);
        cmd.target_actuators[1] = deg2rad(120.0);
        let mut out = [0u8; 256];
        adapter.encode_command(&cmd, &mut out);
        assert_eq!(out[6], 0xFF);
        // -90° knee target below the [-70, …] range → 0x00.
        let mut cmd = CommandVector::zeros(XGO_LEG_MOTORS);
        cmd.target_actuators[0] = deg2rad(-90.0);
        let mut out = [0u8; 256];
        adapter.encode_command(&cmd, &mut out);
        assert_eq!(out[5], 0x00);
    }

    #[test]
    fn velocity_and_action_slots_append_their_frames() {
        let adapter = XgoUart::new();
        let mut cmd = CommandVector::zeros(XGO_LEG_MOTORS + 4);
        cmd.target_actuators[12] = 0.10; // Vx = 10 cm/s
        cmd.target_actuators[14] = deg2rad(30.0); // yaw rate = 30 °/s
        cmd.target_actuators[15] = 1.0; // ACTION macro: stand up
        let mut out = [0u8; 256];
        let n = adapter.encode_command(&cmd, &mut out);
        // Motors (20 B) + velocities (11 B) + action (9 B).
        assert_eq!(n, 20 + 11 + 9);
        assert_eq!(out[20], 0x55);
        assert_eq!(out[22], 11);
        assert_eq!(out[24], REG_VX);
        // 10 cm/s over [-25, 25] → 255/50*(10+25) = 178.5 → 178.
        assert_eq!(out[25], 178);
        // 30 °/s over [-100, 100] → 255/200*130 = 165.75 → 165.
        assert_eq!(out[27], 165);
        assert_eq!(out[31], 0x55);
        assert_eq!(out[34], MODE_WRITE);
        assert_eq!(out[35], REG_ACTION);
        assert_eq!(out[36], 1);
    }

    #[test]
    fn estop_is_the_xgo_reset_action() {
        let adapter = XgoUart::new();
        let mut out = [0u8; 64];
        let n = adapter.encode_estop(XGO_LEG_MOTORS, &mut out);
        assert_eq!(n, 9);
        assert_eq!(
            out[..9],
            [0x55, 0x00, 0x09, 0x00, 0x3E, 0xFF, 0xB9, 0x00, 0xAA][..]
        );
    }

    #[test]
    fn poll_schedule_rotates_over_telemetry_registers() {
        let adapter = XgoUart::new();
        let mut out = [0u8; 64];
        // Phase 0: 12 motor angles. CHK = 255 - (9+2+0x50+12) = 0x98.
        let n = adapter.poll_request(&mut out);
        assert_eq!(n, 9);
        assert_eq!(
            out[..9],
            [0x55, 0x00, 0x09, 0x02, 0x50, 12, 0x98, 0x00, 0xAA][..]
        );
        // Phase 1..3: roll, pitch, yaw (4-byte float reads).
        for addr in [REG_ROLL, REG_PITCH, REG_YAW] {
            let n = adapter.poll_request(&mut out);
            assert_eq!(n, 9);
            assert_eq!(out[4], addr);
            assert_eq!(out[5], 4);
            assert_eq!(out[6], checksum(9, MODE_READ, addr, &[4]));
        }
        // Phase 4: battery (1 byte).
        let n = adapter.poll_request(&mut out);
        assert_eq!(n, 9);
        assert_eq!(out[4], REG_BATTERY);
        assert_eq!(out[5], 1);
        assert_eq!(out[6], checksum(9, MODE_READ, REG_BATTERY, &[1]));
        // Schedule wraps around to the joint angles.
        let n = adapter.poll_request(&mut out);
        assert_eq!(n, 9);
        assert_eq!(out[4], REG_MOTOR_ANGLE);
    }

    #[test]
    fn decodes_motor_angles_roundtrip() {
        let adapter = XgoUart::new();
        // Encode 0.0 rad targets, then feed the same bytes back as a response.
        let cmd = CommandVector::zeros(XGO_LEG_MOTORS);
        let mut out = [0u8; 256];
        adapter.encode_command(&cmd, &mut out);
        let data = out[5..5 + XGO_LEG_MOTORS].to_vec();
        let frame = response_frame(MODE_READ, REG_MOTOR_ANGLE, &data);
        let (consumed, values, n) = adapter.decode_telemetry(&frame, XGO_LEG_MOTORS).unwrap();
        assert_eq!(consumed, frame.len());
        assert_eq!(n, SNAPSHOT_LEN);
        for slot in values.iter().take(XGO_LEG_MOTORS) {
            // u8 quantization keeps the roundtrip well inside 0.01 rad.
            assert!(slot.abs() < 0.01, "{slot}");
        }
    }

    #[test]
    fn decodes_imu_attitude_and_battery() {
        let adapter = XgoUart::new();
        let roll: Vec<u8> = 10.0f32.to_le_bytes().to_vec(); // 10 °
        let (consumed, values, n) = adapter
            .decode_telemetry(
                &response_frame(MODE_READ, REG_ROLL, &roll),
                XGO_LEG_MOTORS,
            )
            .unwrap();
        assert_eq!(consumed, 12);
        assert_eq!(n, SNAPSHOT_LEN);
        assert!((values[XGO_LEG_MOTORS] - deg2rad(10.0)).abs() < 1e-5);

        let battery = response_frame(MODE_READ, REG_BATTERY, &[78]); // 7.8 V
        let (_, values, _) = adapter
            .decode_telemetry(&battery, XGO_LEG_MOTORS)
            .unwrap();
        assert!((values[XGO_LEG_MOTORS + 3] - 7.8).abs() < 1e-5);
    }

    #[test]
    fn imu_nan_is_ignored() {
        let adapter = XgoUart::new();
        let nan: Vec<u8> = f32::NAN.to_le_bytes().to_vec();
        let (_, values, _) = adapter
            .decode_telemetry(
                &response_frame(MODE_READ, REG_ROLL, &nan),
                XGO_LEG_MOTORS,
            )
            .unwrap();
        assert!(values[XGO_LEG_MOTORS].is_finite());
    }

    #[test]
    fn incomplete_frame_waits_for_more_bytes() {
        let adapter = XgoUart::new();
        let full = response_frame(MODE_READ, REG_MOTOR_ANGLE, &[128; XGO_LEG_MOTORS]);
        let decoded = adapter.decode_telemetry(&full[..full.len() - 2], XGO_LEG_MOTORS);
        // Truncated frame: nothing to decode, nothing consumed.
        assert!(decoded.is_none());
    }

    #[test]
    fn garbage_is_flushed_then_the_frame_decodes() {
        let adapter = XgoUart::new();
        let mut stream = vec![0x00, 0xFF, 0x12];
        stream.extend_from_slice(&response_frame(MODE_READ, REG_MOTOR_ANGLE, &[148; 3]));
        // First pass: only the garbage is consumed, no telemetry published.
        let (consumed, _, n) = adapter.decode_telemetry(&stream, XGO_LEG_MOTORS).unwrap();
        assert_eq!((consumed, n), (3, 0));
        // Second pass: the frame decodes.
        let (consumed, values, n) = adapter
            .decode_telemetry(&stream[3..], XGO_LEG_MOTORS)
            .unwrap();
        assert_eq!(consumed, 3 + FRAME_OVERHEAD);
        assert_eq!(n, SNAPSHOT_LEN);
        let expected = deg2rad(conver2float(148, -70.0, 50.0));
        assert!((values[0] - expected).abs() < 1e-5);
    }

    #[test]
    fn corrupt_checksum_resynchronizes() {
        let adapter = XgoUart::new();
        let mut frame = response_frame(MODE_READ, REG_MOTOR_ANGLE, &[128; 3]);
        let last = frame.len() - 3;
        frame[last] ^= 0xFF; // break the checksum
        let (consumed, _, n) = adapter.decode_telemetry(&frame, XGO_LEG_MOTORS).unwrap();
        assert_eq!((consumed, n), (2, 0)); // fake header pair dropped
    }
}
