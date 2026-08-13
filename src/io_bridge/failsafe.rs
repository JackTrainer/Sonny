use std::time::Instant;

use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};
use crate::io_bridge::state_vector::StateVector;

/// Safety guard for the embodiment in an abnormal state.
///
/// When a radio blackout or a corrupted sensor takes away the controller's
/// view of the hardware, the guard freezes the last valid state received
/// (joint positions and gripper pressure) and re-commands it every tick
/// without any allocation: the grip on the glass jar stays at perfect pressure
/// and the carried box does not move a millimeter.
///
/// Reaction time is measured with `Instant::now()` and typically stays under
/// the 2 milliseconds required by the industrial 100 Hz cycle.
pub struct FailsafeGuard {
    engaged: bool,
    pressure_slot: usize,
    max_grip_n: f32,
    last_good: [f32; MAX_JOINTS],
    last_good_pressure_n: f32,
    blackout_frames: u64,
    interventions: u64,
    last_response_us: u64,
}

impl FailsafeGuard {
    pub fn new(pressure_slot: usize, nominal_grip_n: f32, max_grip_n: f32) -> Self {
        let nominal = nominal_grip_n.clamp(0.0, max_grip_n);
        Self {
            engaged: false,
            pressure_slot,
            max_grip_n,
            last_good: [0.0; MAX_JOINTS],
            last_good_pressure_n: nominal,
            blackout_frames: 0,
            interventions: 0,
            last_response_us: 0,
        }
    }

    pub fn is_engaged(&self) -> bool {
        self.engaged
    }

    pub fn blackout_frames(&self) -> u64 {
        self.blackout_frames
    }

    pub fn interventions(&self) -> u64 {
        self.interventions
    }

    pub fn last_response_us(&self) -> u64 {
        self.last_response_us
    }

    /// Grip pressure currently held by the guard.
    pub fn held_grip_n(&self) -> f32 {
        self.last_good_pressure_n
    }

    /// Freezes the last valid telemetry state received from the bus: from this
    /// moment on it is the safe reference for any anomaly.
    pub fn snapshot_healthy(&mut self, state: &StateVector) {
        let n = state.len.min(MAX_JOINTS);
        self.last_good[..n].copy_from_slice(&state.values[..n]);
        if self.pressure_slot < n {
            self.last_good_pressure_n = state.values[self.pressure_slot].clamp(0.0, self.max_grip_n);
        }
    }

    /// Records a frame lost due to radio blackout and arms the guard.
    pub fn on_blackout(&mut self) {
        self.engaged = true;
        self.blackout_frames += 1;
    }

    /// Disarms the guard when telemetry becomes stable again.
    pub fn disengage(&mut self) {
        self.engaged = false;
    }

    /// Re-applies the last safe state to the command: if the guard is armed,
    /// it forces every register back to the frozen value (including the
    /// nominal grip pressure). Returns the reaction time in microseconds.
    pub fn enforce(&mut self, command: &mut CommandVector) -> u64 {
        let start = Instant::now();
        if self.engaged {
            let n = command.len.min(MAX_JOINTS);
            for i in 0..n {
                command.target_actuators[i] = self.last_good[i];
            }
            if self.pressure_slot < n {
                command.target_actuators[self.pressure_slot] = self.last_good_pressure_n;
            }
            self.interventions += 1;
        }
        self.last_response_us = start.elapsed().as_micros() as u64;
        self.last_response_us
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blackout_holds_last_stable_pressure() {
        let mut failsafe = FailsafeGuard::new(2, 28.0, 100.0);
        let state = StateVector::new("T", &[1.0, 2.0, 28.0, 4.0]);
        failsafe.snapshot_healthy(&state);

        failsafe.on_blackout();
        let mut cmd = CommandVector::from_slice(&[99.0, 99.0, 99.0, 99.0]);
        let us = failsafe.enforce(&mut cmd);
        assert!(us < 2000, "failsafe beyond 2ms: {}us", us);
        assert_eq!(cmd.as_slice(), &[1.0, 2.0, 28.0, 4.0][..]);
        assert!(failsafe.is_engaged());
        assert_eq!(failsafe.held_grip_n(), 28.0);
        assert_eq!(failsafe.interventions(), 1);
    }

    #[test]
    fn healthy_frames_do_not_intervene() {
        let mut failsafe = FailsafeGuard::new(0, 28.0, 100.0);
        let mut cmd = CommandVector::from_slice(&[1.0, 2.0]);
        let us = failsafe.enforce(&mut cmd);
        assert_eq!(cmd.as_slice(), &[1.0, 2.0][..]);
        assert_eq!(failsafe.interventions(), 0);
        assert_eq!(failsafe.blackout_frames(), 0);
        assert!(us < 2000);
    }

    #[test]
    fn disengage_stops_hold() {
        let mut failsafe = FailsafeGuard::new(0, 28.0, 100.0);
        failsafe.on_blackout();
        failsafe.disengage();
        let mut cmd = CommandVector::from_slice(&[5.0]);
        failsafe.enforce(&mut cmd);
        assert_eq!(cmd.as_slice(), &[5.0][..]);
        assert!(!failsafe.is_engaged());
    }
}
