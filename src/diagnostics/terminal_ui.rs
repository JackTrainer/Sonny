use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::time::sleep;

const BAR_WIDTH: usize = 16;
const MAX_WARNINGS: usize = 8;

/// State shared between the telemetry/status subscription task and the
/// render loop: updated without blocking by the bus loop, read at 5 Hz.
#[derive(Debug, Clone)]
pub struct JointView {
    pub name: String,
    pub value: f32,
    pub min: f32,
    pub max: f32,
}

#[derive(Debug)]
pub struct UiState {
    pub online: bool,
    pub transport: String,
    pub estop: bool,
    pub frames: u64,
    pub last_interval_us: Option<u64>,
    pub max_jitter_us: u64,
    pub last_frame_at: Option<Instant>,
    pub joints: Vec<JointView>,
    pub warnings: Vec<String>,
}

impl UiState {
    pub fn new(joint_names: &[String], joint_limits: &[(f32, f32)]) -> Self {
        let default_range = (-std::f32::consts::PI, std::f32::consts::PI);
        let joints = joint_names
            .iter()
            .enumerate()
            .map(|(i, name)| {
                let (min, max) = joint_limits.get(i).copied().unwrap_or(default_range);
                JointView {
                    name: name.clone(),
                    value: 0.0,
                    min,
                    max,
                }
            })
            .collect();
        Self {
            online: false,
            transport: "OFFLINE".into(),
            estop: false,
            frames: 0,
            last_interval_us: None,
            max_jitter_us: 0,
            last_frame_at: None,
            joints,
            warnings: Vec::new(),
        }
    }

    pub fn push_warning(&mut self, msg: String) {
        self.warnings.insert(0, msg);
        self.warnings.truncate(MAX_WARNINGS);
    }
}

#[derive(Clone)]
pub struct TerminalUi {
    session: Arc<zenoh::Session>,
    robot_name: String,
    expected_hz: f64,
    joint_names: Vec<String>,
    joint_limits: Vec<(f32, f32)>,
}

impl TerminalUi {
    pub fn new(
        session: Arc<zenoh::Session>,
        robot_name: &str,
        expected_hz: f64,
        joint_names: Vec<String>,
        joint_limits: Vec<(f32, f32)>,
    ) -> Self {
        Self {
            session,
            robot_name: robot_name.to_string(),
            expected_hz,
            joint_names,
            joint_limits,
        }
    }

    /// Starts the data update task and the 5 Hz rendering loop.
    pub async fn start_render_loop(&self) {
        println!("[DIAGNOSTICS] Starting SONNY Terminal UI...");

        let state = self.spawn_updater();
        self.render_loop(state, Duration::from_millis(200)).await;
    }

    /// Creates the shared dashboard state and starts the task that feeds it
    /// by subscribing to telemetry/status on the Zenoh bus.
    pub fn spawn_updater(&self) -> Arc<Mutex<UiState>> {
        let state = Arc::new(Mutex::new(UiState::new(
            &self.joint_names,
            &self.joint_limits,
        )));

        let updater = self.clone();
        let updater_state = state.clone();
        tokio::spawn(async move {
            updater.run_telemetry_updater(updater_state).await;
        });

        state
    }

    /// Dashboard rendering loop with the requested refresh period.
    pub async fn render_loop(&self, state: Arc<Mutex<UiState>>, interval: Duration) {
        loop {
            self.render(&state);
            sleep(interval).await;
        }
    }

    // -----------------------------------------------------------------------
    // Data update task from the bus
    // -----------------------------------------------------------------------
    async fn run_telemetry_updater(&self, state: Arc<Mutex<UiState>>) {
        let telemetry_topic = "alpha/telemetry/*";
        let status_topic = "alpha/status/*";

        let telemetry_sub = match self.session.declare_subscriber(telemetry_topic).await {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[UI] Telemetry subscriber failed: {}", e);
                return;
            }
        };
        let status_sub = match self.session.declare_subscriber(status_topic).await {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[UI] Status subscriber failed: {}", e);
                return;
            }
        };

        let expected_interval_us = if self.expected_hz > 0.0 {
            (1_000_000.0 / self.expected_hz) as u64
        } else {
            u64::MAX
        };

        loop {
            tokio::select! {
                sample = telemetry_sub.recv_async() => {
                    if let Ok(sample) = sample {
                        let topic = sample.key_expr().to_string();
                        if Self::tail_matches(&topic, &self.robot_name) {
                            let payload = sample.payload().to_bytes();
                            if payload.len() % 4 == 0 {
                                let mut guard = state.lock().unwrap();
                                self.apply_telemetry(&mut guard, &payload, expected_interval_us);
                            }
                        }
                    }
                }
                sample = status_sub.recv_async() => {
                    if let Ok(sample) = sample {
                        let topic = sample.key_expr().to_string();
                        if Self::tail_matches(&topic, &self.robot_name) {
                            let payload = sample.payload().to_bytes();
                            if let Ok(json) = serde_json::from_slice::<serde_json::Value>(&payload) {
                                let mut guard = state.lock().unwrap();
                                self.apply_status(&mut guard, &json);
                            }
                        }
                    }
                }
            }
        }
    }

    fn tail_matches(topic: &str, robot_name: &str) -> bool {
        topic.rsplit('/').next() == Some(robot_name)
    }

    fn apply_telemetry(&self, state: &mut UiState, payload: &[u8], expected_interval_us: u64) {
        let values: Vec<f32> = payload
            .chunks_exact(4)
            .map(|c| f32::from_le_bytes(c.try_into().unwrap()))
            .collect();

        for (i, joint) in state.joints.iter_mut().enumerate() {
            if let Some(v) = values.get(i) {
                joint.value = *v;
            }
        }

        let now = Instant::now();
        if let Some(last) = state.last_frame_at {
            let interval_us = now.duration_since(last).as_micros() as u64;
            // Anti-burst guard: samples delivered in bursts by the bus are not
            // real telemetry intervals and would pollute the estimate.
            if interval_us >= 500 {
                state.last_interval_us = Some(interval_us);
                if interval_us > state.max_jitter_us {
                    state.max_jitter_us = interval_us;
                }
                let diff_us = if interval_us > expected_interval_us {
                    interval_us - expected_interval_us
                } else {
                    expected_interval_us - interval_us
                };
                if expected_interval_us != u64::MAX && diff_us > expected_interval_us * 5 / 2 {
                    state.push_warning(format!(
                        "Jitter anomalo: intervallo {} us (atteso ~{} us)",
                        interval_us, expected_interval_us
                    ));
                }
            }
        }
        state.last_frame_at = Some(now);
        state.frames += 1;
    }

    fn apply_status(&self, state: &mut UiState, json: &serde_json::Value) {
        if let Some(online) = json.get("online").and_then(|v| v.as_bool()) {
            state.online = online;
        }
        if let Some(transport) = json.get("transport").and_then(|v| v.as_str()) {
            state.transport = transport.to_string();
        }
        if let Some(estop) = json.get("estop").and_then(|v| v.as_bool()) {
            state.estop = estop;
        }
        if let Some(frames) = json.get("frames").and_then(|v| v.as_u64()) {
            if frames > state.frames {
                state.frames = frames;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Rendering
    // -----------------------------------------------------------------------
    pub fn render(&self, state: &Arc<Mutex<UiState>>) {
        let state = state.lock().unwrap();

        print!("{}[2J{}[H", 27 as char, 27 as char);

        println!("====================================================");
        println!("      SONNY - OPERATING SYSTEM DASHBOARD");
        println!("====================================================");
        println!(" HARDWARE NODE:    {}", self.robot_name);

        let status = if state.estop {
            "ESTOP"
        } else if state.online {
            "ONLINE"
        } else {
            "OFFLINE"
        };
        let status_color = match status {
            "ESTOP" => "\x1b[1;31m",
            "ONLINE" => "\x1b[1;32m",
            _ => "\x1b[1;33m",
        };
        println!(
            " FIELD BUS:        {}{}{}  [{}]",
            status_color, status, "\x1b[0m", state.transport
        );

        let (hz, interval_ms) = match state.last_interval_us {
            Some(us) => (1_000_000.0 / us as f64, us as f64 / 1000.0),
            None => (0.0, 0.0),
        };
        println!(" TELEMETRY RATE:   {:.1} Hz (expected {:.1}) | interval {:.2} ms | max jitter {:.2} ms",
            hz, self.expected_hz, interval_ms, state.max_jitter_us as f64 / 1000.0);
        println!(" FRAMES RECEIVED:  {}", state.frames);
        println!("----------------------------------------------------");

        if state.joints.is_empty() {
            println!(" (no telemetry received yet)");
        }
        for joint in &state.joints {
            let bar = Self::joint_bar(joint);
            println!(" {:<14} {:>+7.3} rad  {}", joint.name, joint.value, bar);
        }

        println!("----------------------------------------------------");
        println!(
            " COMMANDS:   alpha/cmd/{}        (f32 LE x DOF)",
            self.robot_name
        );
        println!(
            " ESTOP:     alpha/cmd/{}/estop (1 = active)",
            self.robot_name
        );
        println!("====================================================");

        for warning in state.warnings.iter().take(3) {
            println!(" [WARN] {}", warning);
        }
        if state.warnings.is_empty() {
            println!(" NO ALARMS");
        }
        println!("====================================================\n");
    }

    fn joint_bar(joint: &JointView) -> String {
        let span = joint.max - joint.min;
        if span <= f32::EPSILON {
            return "[]".into();
        }
        let pos = ((joint.value - joint.min) / span).clamp(0.0, 1.0);
        let idx = (pos * (BAR_WIDTH - 1) as f32).round() as usize;
        let mut s = String::with_capacity(BAR_WIDTH + 2);
        s.push('[');
        for i in 0..BAR_WIDTH {
            if i == idx {
                s.push('X');
            } else {
                s.push('-');
            }
        }
        s.push(']');
        s
    }
}
