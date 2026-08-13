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
            "[REGISTRY] Ricevuto nuovo pacchetto binario .skill di {} byte.",
            wasm_bytecode.len()
        );
        execute_skill_sandbox(&wasm_bytecode, output.clone());
    }
    Ok(())
}

/// Esegue la Skill in un sandbox dedicato e ne pubblica il risultato nel
/// buffer lock-free a capacità 1.
///
/// Il runtime Wasmtime vive sul Core 2, isolato dal loop di controllo che
/// gira sul Core 1: anche se una Skill impiega centinaia di microsecondi, il
/// Core 1 non lo aspetta mai e riusa l'ultimo vettore di coordinate valido.
fn execute_skill_sandbox(bytecode: &[u8], output: Arc<LatestCommand>) {
    let bytecode = bytecode.to_vec();
    std::thread::Builder::new()
        .name("wasmtime-runtime".into())
        .spawn(move || {
            if core_affinity::set_for_current(core_affinity::CoreId { id: WASM_RUNTIME_CORE }) {
                println!(
                    "[SANDBOX] Runtime WASM pinnato sul Core {}.",
                    WASM_RUNTIME_CORE
                );
            } else {
                eprintln!(
                    "[WARN - SANDBOX] Pinning del runtime WASM sul Core {} non riuscito.",
                    WASM_RUNTIME_CORE
                );
            }

            println!("[SANDBOX] Inizializzazione ambiente virtuale isolato (WASM)...");

            // NOTA PER LA COMMUNITY: Questa è l'interfaccia cieca di isolamento.
            // La ricompensa a tre stati (-1.0, 0.0, 1.0) viene valutata qui dentro
            // preservando la sicurezza hardware e la protezione del codice aziendale.

            let reward = evaluate_reward(&bytecode);

            // Il sandbox produce coordinate geometriche: il Core 1 le legge dal
            // buffer senza mai bloccarsi, oppure riusa l'ultimo valore valido.
            let cmd = CommandVector::from_slice(&[reward, -reward, reward * 0.5]);
            output.publish(cmd);

            println!(
                "[SANDBOX] Esecuzione logica di ricompensa completata (ricompensa {}). Comando pubblicato sul buffer lock-free.",
                reward
            );
        })
        .expect("spawn del thread del runtime WASM");
}

/// Valutazione della ricompensa della Skill: qui entrerà il motore Wasmtime
/// vero e proprio. Il mock restituisce una ricompensa deterministica a tre
/// stati in funzione del bytecode, senza mai eseguire codice arbitrario.
fn evaluate_reward(bytecode: &[u8]) -> f32 {
    match bytecode.len() % 3 {
        0 => 0.0,
        1 => -1.0,
        _ => 1.0,
    }
}
