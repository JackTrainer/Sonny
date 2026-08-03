<img width="1983" height="793" alt="image" src="https://github.com/user-attachments/assets/bf806f8d-8a9b-4dcb-b1c6-c6eb8ad35700" />

# 🪐 SONNY OS — Core Minimal Microkernel

[![License: AGPL v3](https://shields.io)](https://gnu.org)
[![Language: Rust](https://shields.io)](https://rust-lang.org)
[![Network: Zenoh](https://shields.io)](https://zenoh.io)

**SONNY OS** is an ultra-lightweight, high-performance, universally agnostic microkernel built to redefine how physical hardware interfaces with computing intelligence. Written entirely in native **Rust**, it replaces legacy, bloated, and unstable robotic frameworks (such as ROS 2) with a deterministic asynchronous real-time architecture. 

This repository contains the **Core Minimal Open-Source Version**: the foundational infrastructure, universal hardware abstraction layer (HAL), real-time diagnostics, and ultra-low latency messaging tubes. It is designed to act as a universal plug-and-play backbone for **any robotic embodiment**—including industrial picking arms, AMRs, quadrupeds, drones, or humanoids.

---

## ⚡ The Architectural Battle: SONNY OS vs. Brute-Force AI

The current Silicon Valley paradigm (led by companies like **Physical Intelligence** with their π₀ models) burns millions of dollars daily in heavy GPU cloud clusters to train giant statistics-based networks. Their approach attempts to "guess" physical manipulation by imitating internet videos via brute-force token matching. 

**The Result?** It takes **200 to 500 hours of human data collection and 12 to 24 hours of massive cloud computing** to teach a robotic arm a single new skill. The deployment cycle takes weeks, and the resulting black-box models frequently hallucinate or trigger fatal segmentation faults whenever factory lighting or item transparency changes.

**SONNY OS changes the rules of physical automation:**
* **Physics is a Calculation, Not a Guess:** Instead of brute-force data training, SONNY OS interfaces with a proprietary structural causal model based on Pearl's *Do-calculus*. 
* **From Weeks to 10 Minutes:** SONNY OS enables robots to master complex physical skills in minutes using a single first-person (POV) video and local edge simulation.
* **8.5x Resource Efficiency:** Under severe 65% wireless packet loss, traditional ROS 2 DDS queues overflow and crash. SONNY OS remains perfectly stable at a steady 100Hz loop, consuming 8.5 times less memory overhead.


## 🛠️ The Core Fix: Solving the 3 Ultimate Nightmares of ROS 2 Engineers

Every robotics engineer knows the pain of maintaining a production-grade ROS 2 stack. SONNY OS was engineered from day one to architecturally eliminate these legacy bottlenecks:

<img width="1413" height="481" alt="image" src="https://github.com/user-attachments/assets/f16f5232-9c4b-4e22-b87a-816ff4224767" />


### 1. Zero Dependency Purgatory (Single-Binary Native Execution)
* **The ROS 2 Nightmare:** Deploying ROS 2 requires a highly specific Ubuntu flavor, complex environmental sourcing, CMake synchronization, Python runtime dependencies, and an endless cycle of compilation crashes.
* **The SONNY OS Fix:** SONNY OS compiles down to a single, hyper-optimized, standalone native binary. With zero garbage collection and no external runtime requirements, you type `cargo run` and it executes flawlessly across any architecture—be it a Windows workstation, an x86 Linux server, or an ultra-low-power ARM-based NVIDIA Jetson edge node. 

### 2. High-Frequency Memory Stability (8.5x Overhead Reduction vs. DDS)
* **The ROS 2 Nightmare:** ROS 2 relies on Data Distribution Service (DDS) middleware, which forces massive XML/IDL serialization on the heap. If a mobile robot or drone hits a weak Wi-Fi spot in a warehouse, un-sent DDS messages saturate the memory queue, triggering an un-recoverable `Segmentation Fault` that freezes the physical machine.
* **The SONNY OS Fix:** SONNY OS replaces DDS with a flat, real-time memory bus using stack-allocated static arrays combined with the ultra-lightweight **Zenoh** protocol. Network overhead is cut down to a fixed 5 bytes per packet. If connection drops, Zenoh manages backpressure natively at the edge without a single memory allocation, ensuring the robot never crashes or loses positional tracking.

### 3. Declarative Hardware Isolation (No More Recoding Drivers)
* **The ROS 2 Nightmare:** Swapping a motor, changing a joint limit, or adding a new tactile sensor forces developers to restructure URDF files, rewrite custom C++ hardware interfaces, and re-compile entire node packages, risking unexpected behavior across the rest of the kinematic chain.
* **The SONNY OS Fix:** SONNY OS decouples hardware from the control loops completely through its declarative `OpenHalConfig` JSON loader. Changing physical joint limits or register addresses is a 2-second configuration task. The microkernel maps the new parameters to the universal `StateVector` at runtime, ensuring complete operational isolation. Your core behavioral code remains completely untouched.

---

## 🧩 How It Works: Universal Hardware Abstraction

SONNY OS reduces any mechanical complexity or joint geometry into a standardized linear mathematical vector (`Vec<f32>`). Hardware developers and engineers do not need to rewrite the core operating system or complex driver layers; integration is entirely **declarative** and achieved via a single, human-readable JSON configuration file (`OpenHalConfig`).

1. **`io_bridge`**: Linearizes any joint inputs (radians, torques, pressures) into continuous array buffers and compresses commands to the actuators with only 5 bytes of network overhead.
2. **`diagnostics`**: Includes an active `FrequencyEnforcer` and a real-time hard-watchdog loop that monitors sub-millisecond jitter to intercept timing delays before hardware collisions occur.
3. **`registry`**: Provides a secure execution sandbox powered by **WebAssembly (WASM)**, isolating high-level skill policies from lower-level system critical registers.

---

## 🤖 Hardware Compatibility

SONNY OS is compatible out-of-the-box with the major industrial robot brands. The brand is auto-detected from the `manufacturer` field of the HAL JSON (or forced via the optional `brand` field); each profile selects a native protocol codec in `src/io_bridge/adapters/` while the kernel stays byte-for-byte identical.

| Brand | Profile | Native protocol | Sample config |
|-------|---------|-----------------|---------------|
| KUKA | `KUKA` | Ethernet KRL (XML over TCP, port 7911) | `configs/kuka_kr6_r900.json` |
| Comau | `COMAU` | PDL2 socket bridge (CSV over TCP) | `configs/comau_racer5.json` |
| Universal Robots | `UNIVERSAL-ROBOTS` | RT Client (TCP 30003, binary) + URScript (30002) | `configs/universal_robots_ur5e.json` |
| Franka Emika | `FRANKA-EMIKA` | FCI (UDP 30401, 1 kHz) | `configs/franka_emika_panda.json` |
| Standard AMR | `AMR` | ROS2 Twist/Odom mapping (differential) | `configs/amr_standard_base.json` |
| Any other | `GENERIC` | Fixed f32 LE binary frame | `configs/robot_6dof_anthropomorphic.json` |

Telemetry from the robot (whatever the brand) is normalized into the same fixed `[f32; 32]` state vector and published on `alpha/telemetry/{id}`; commands published on `alpha/cmd/{id}` are encoded into the brand-native protocol by the adapter. Safety (joint-limit clamping, ESTOP, hard watchdog) applies before encoding, in brand-independent vector space.

```bash
cargo run --release -- configs/kuka_kr6_r900.json
```

---

## 📁 Repository Structure

```text
SONNY/
├── LICENSE                         # GNU AGPLv3 Copyleft License
├── README.md                       # Documentation
├── Cargo.toml                      # Native dependencies (Tokio, Zenoh, Serde, libc)
├── Cargo.lock
├── configs/                        # Declarative robot profiles (OpenHalConfig JSON)
│   ├── amr_standard_base.json
│   ├── comau_racer5.json
│   ├── franka_emika_panda.json
│   ├── kuka_kr6_r900.json
│   ├── robot_6dof_anthropomorphic.json
│   └── universal_robots_ur5e.json
└── src/
    ├── main.rs                     # Async bootloader, RT thread pinning, fieldbus resolution
    ├── lib.rs                      # Crate root, public module exports
    ├── diagnostics/
    │   ├── mod.rs
    │   ├── frequency_enforcer.rs   # 100 Hz jitter monitor against control deadlines
    │   ├── telemetry_logger.rs     # Telemetry serialization on the fixed state vector
    │   └── terminal_ui.rs          # Live real-time terminal dashboard
    ├── io_bridge/
    │   ├── mod.rs
    │   ├── adapters/
    │   │   ├── mod.rs
    │   │   ├── amr_ros2.rs         # ROS2 Twist/Odom mapping (differential AMR)
    │   │   ├── comau_pdl.rs        # PDL2 socket bridge (CSV over TCP)
    │   │   ├── franka_fci.rs       # FCI UDP 30401 (1 kHz)
    │   │   ├── generic_binary.rs   # Fixed f32 LE binary frame
    │   │   ├── kuka_eth_krl.rs     # Ethernet KRL (XML over TCP 7911)
    │   │   └── universal_robots.rs # RT Client TCP 30003 + URScript 30002
    │   ├── command_vector.rs       # Fixed [f32; MAX_JOINTS] command buffer
    │   ├── hal_loader.rs           # OpenHalConfig JSON loader -> FieldbusConfig
    │   ├── hardware_abstraction.rs # Fieldbus loops (Serial/UDP/TCP) + clamp_command
    │   ├── jerk_limiter.rs         # 3rd-order jerk/accel/speed saturation chain
    │   ├── robot_brand.rs          # Brand auto-detection & profile selection
    │   ├── state_vector.rs         # Fixed [f32; MAX_JOINTS] telemetry state
    │   ├── vector_sanitizer.rs     # NaN/Inf replacement + geometric clamp
    │   └── watchdog_timer.rs       # HardWatchdog heartbeat + E-stop
    ├── math_utils/
    │   ├── mod.rs
    │   └── kinematics_math.rs      # Spatial vector math, kinematics, quaternions
    ├── mocks/
    │   ├── mod.rs
    │   └── virtual_hardware_mock.rs # Sinusoidal virtual hardware simulator
    ├── network_hook/
    │   ├── mod.rs
    │   ├── telemetry_client.rs     # Zero-copy telemetry client
    │   └── zenoh_bus.rs            # Zenoh unified ultra-low-latency bus
    ├── registry/
    │   ├── mod.rs
    │   └── wasm_runner.rs          # WASM isolated skill execution sandbox
    ├── ros2_bridge/
    │   ├── mod.rs
    │   └── topic_converter.rs      # ROS2 topics <-> SONNY vector conversion
    └── system/
        ├── mod.rs
        └── rt_thread.rs            # CPU affinity + SCHED_FIFO / nice -20 pinning
```

---

## 🛠️ Getting Started: Run SONNY on a Real Robot

### Prerequisites
* **Rust stable** toolchain on the edge device that talks to the robot (laptop, Jetson, Raspberry Pi) or directly on the robot's Linux controller.
* The robot controller reachable on the network (or a serial cable plugged in).
* A **Linux** host for the real-time thread isolation (`SCHED_FIFO` + CPU affinity). On Windows/macOS SONNY still runs, but the real-time pinning is a no-op.

### 1. Point the Config at Your Robot Controller

Every robot profile lives in `configs/`. Open the JSON of your arm and set the `fieldbus` block so it matches your physical controller — the IP/port below are placeholders:

```json
"fieldbus": {
  "protocol": "Tcp",
  "remote_addr": "192.168.1.2:7911",
  "frame_timeout_ms": 100,
  "reconnect_delay_ms": 2000
}
```

The `brand` is auto-detected from `manufacturer` (or forced with `brand`) and selects the native codec: `KUKA` (Ethernet KRL / TCP 7911), `UNIVERSAL-ROBOTS` (RT Client / TCP 30003), `COMAU` (PDL2 / TCP 6000), `FRANKA-EMIKA` (FCI / UDP 30401), `AMR` (ROS2 Twist-Odom), `GENERIC` (fixed f32 binary).

### 2. Compile the Release Binary

```bash
git clone https://github.com/JackTrainer/Sonny.git
cd Sonny
cargo build --release
```

### 3. Run It on the Real Robot

Pass the config path as the first argument:

```bash
./target/release/SONNY configs/kuka_kr6_r900.json
```

(or without building separately: `cargo run --release -- configs/kuka_kr6_r900.json`).

You should see `[BOOT] Sistema pronto su hardware: KUKA-KR6-R900 (profilo: KUKA)` and the live 100 Hz terminal dashboard. `Ctrl+C` triggers a clean shutdown with E-stop/watchdog release.

### 4. Real-Time Priority (Recommended on the Edge Node)

At startup SONNY pins the control thread to a dedicated core and asks the kernel for `SCHED_FIFO` real-time priority. That requires `CAP_SYS_NICE` — grant it once to the binary so it can elevate priority without running as root:

```bash
sudo setcap cap_sys_nice=ep target/release/SONNY
./target/release/SONNY configs/kuka_kr6_r900.json
```

Without the capability the kernel refuses `SCHED_FIFO` and SONNY automatically falls back to niceness `-20` (log line `[WARN - RT] SCHED_FIFO rifiutata (...): fallback niceness -20 attivo`).

### 5. Quick Starts per Transport

**Ethernet robot (TCP)** — KUKA, Comau, Universal Robots:
```bash
./target/release/SONNY configs/comau_racer5.json
```

**Franka Emika (UDP FCI, 1 kHz)**:
```bash
./target/release/SONNY configs/franka_emika_panda.json
```

**Serial fieldbus (Modbus RTU / CAN via RS-232/485)** — set the port in the JSON, or override everything via environment:
```bash
ROBOT_SERIAL_PORT=/dev/ttyUSB0 \
ROBOT_SERIAL_BAUD=115200 \
ROBOT_NAME=MyArm \
ROBOT_DOF=6 \
./target/release/SONNY
```

**UDP robot via environment** (bind address + robot endpoint):
```bash
ROBOT_UDP_BIND=0.0.0.0:0 \
ROBOT_UDP_REMOTE=192.168.1.2:30401 \
./target/release/SONNY
```

**Remote telemetry across machines**: telemetry and commands travel over Zenoh on `alpha/telemetry/{id}` / `alpha/cmd/{id}`. To stream to another PC, point the node at your Zenoh router with a config file:
```bash
ZENOH_CONFIG=zenoh/router.json ./target/release/SONNY configs/universal_robots_ur5e.json
```

> ⚠️ Safety: the Hard-Watchdog arms on boot and stops the actuators if the control heartbeat misses its deadline. Always test in a fenced, speed-limited environment with the E-stop reachable before commanding real motion.

---

## 🔏 Strategic Scope & Commercial License

**Please Note:** This repository is strictly the **Core Minimal Shell** of SONNY OS. It provides the essential communication tubes, telemetry data serialization, and sandbox runtimes. It **does not** contain the proprietary cloud compiler, the structural causal engine, or the advanced multi-scale swarm synchronization.

* **Open-Source Contribution:** We welcome global contributions to expand the HAL JSON drivers for specific robot models. This core is licensed under the **GNU Affero General Public License v3 (AGPLv3)**. Any commercial use or modifications of this infrastructure must remain public and open-source under the same terms.
* **Enterprise Beta:** To unlock the full Causal Engine, the 3-minute local "Dreaming" optimization loops, and 24/7 mission-critical production SLAs for 3PL and fulfillment centers without open-source copyleft restrictions, visit our official platform - https://alpha-robotics.it/
