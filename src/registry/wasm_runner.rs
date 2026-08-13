use std::sync::Arc;

use crate::diagnostics::frequency_enforcer::{LatestCommand, WASM_RUNTIME_CORE};
use crate::io_bridge::command_vector::CommandVector;

pub async fn listen_for_skills(
    session: Arc<zenoh::Session>,
    output: Arc<LatestCommand>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let subscriber = session
        .declare_subscriber("alpha/robot/local/skills/inject")
        .await?;

    while let Ok(sample) = subscriber.recv_async().await {
        let wasm_bytecode = sample.payload().to_bytes().to_vec();
        println!(
            "[REGISTRY] Received new .skill binary package of {} bytes.",
            wasm_bytecode.len()
        );
        execute_skill_sandbox(&wasm_bytecode, output.clone());
    }
    Ok(())
}

/// Runs the Skill in a dedicated sandbox and publishes its result into the
/// capacity-1 lock-free buffer.
///
/// The Wasmtime runtime lives on Core 2, isolated from the control loop
/// running on Core 1: even if a Skill takes hundreds of microseconds, Core 1
/// never waits for it and reuses the last valid coordinate vector.
fn execute_skill_sandbox(bytecode: &[u8], output: Arc<LatestCommand>) {
    let bytecode = bytecode.to_vec();
    std::thread::Builder::new()
        .name("wasmtime-runtime".into())
        .spawn(move || {
            if core_affinity::set_for_current(core_affinity::CoreId { id: WASM_RUNTIME_CORE }) {
                println!(
                    "[SANDBOX] WASM runtime pinned to Core {}.",
                    WASM_RUNTIME_CORE
                );
            } else {
                eprintln!(
                    "[WARN - SANDBOX] Failed to pin WASM runtime to Core {}.",
                    WASM_RUNTIME_CORE
                );
            }

            println!("[SANDBOX] Initializing isolated virtual environment (WASM)...");

            // NOTE TO THE COMMUNITY: This is the blind isolation interface.
            // The three-state reward (-1.0, 0.0, 1.0) is evaluated here,
            // preserving hardware safety and corporate code protection.

            let reward = evaluate_reward(&bytecode);

            // The sandbox produces geometric coordinates: Core 1 reads them from
            // the buffer without ever blocking, or reuses the last valid value.
            let cmd = CommandVector::from_slice(&[reward, -reward, reward * 0.5]);
            output.publish(cmd);

            println!(
                "[SANDBOX] Reward logic execution completed (reward {}). Command published on the lock-free buffer.",
                reward
            );
        })
        .expect("spawn of the WASM runtime thread");
}

/// Skill reward evaluation: the actual Wasmtime engine will be plugged in
/// here. The mock returns a deterministic three-state reward based on the
/// bytecode, without ever executing arbitrary code.
fn evaluate_reward(bytecode: &[u8]) -> f32 {
    match bytecode.len() % 3 {
        0 => 0.0,
        1 => -1.0,
        _ => 1.0,
    }
}
