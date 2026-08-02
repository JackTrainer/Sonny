
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
├── Cargo.toml                      # Native dependencies (Tokio, Zenoh, Serde)
└── src/
    ├── main.rs                     # Asynchronous bootloader & kernel initialization
    ├── io_bridge/                  # Universal hardware vector linearizer & JSON HAL loader
    ├── math_utils/                 # Spatial vector math, kinematics, and quaternions
    ├── diagnostics/                # Real-time Terminal UI, logger, and 100Hz jitter monitor
    ├── mocks/                      # Sinusoidal virtual hardware simulator for rapid testing
    ├── registry/                   # Secure WebAssembly (WASM) isolated skill execution sandbox
    └── network_hook/               # Zero-copy telemetry clients and Zenoh unifiers
```

---

## 🛠️ Getting Started via Terminal

### Prerequisites
Ensure you have the latest stable Rust toolchain and Cargo installed on your edge device or laptop.

### 1. Clone and Compile the Microkernel
Clone this repository and compile the native core in maximum optimization mode:
```bash
git clone https://github.com
cd SONNY
cargo build --release
```

### 2. Run the Diagnostic Simulator
Execute the bootloader with un-captured stdout logs to visualize the real-time 100Hz memory bus and terminal graphics dashboard:
```bash
cargo run --release -- --nocapture
```
The microkernel will spin up an isolated, virtual asynchronous runtime instance, self-correcting micro-timing delays and initializing local Zenoh streaming nodes automatically.

---

## 🔏 Strategic Scope & Commercial License

**Please Note:** This repository is strictly the **Core Minimal Shell** of SONNY OS. It provides the essential communication tubes, telemetry data serialization, and sandbox runtimes. It **does not** contain the proprietary cloud compiler, the structural causal engine, or the advanced multi-scale swarm synchronization.

* **Open-Source Contribution:** We welcome global contributions to expand the HAL JSON drivers for specific robot models. This core is licensed under the **GNU Affero General Public License v3 (AGPLv3)**. Any commercial use or modifications of this infrastructure must remain public and open-source under the same terms.
* **Enterprise Beta:** To unlock the full Causal Engine, the 3-minute local "Dreaming" optimization loops, and 24/7 mission-critical production SLAs for 3PL and fulfillment centers without open-source copyleft restrictions, visit our official platform - https://alpha-robotics.it/
