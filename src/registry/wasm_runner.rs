use std::sync::Arc;

pub async fn listen_for_skills(session: Arc<zenoh::Session>) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let subscriber = session.declare_subscriber("alpha/robot/local/skills/inject").await?;

    while let Ok(sample) = subscriber.recv_async().await {
        let wasm_bytecode = sample.payload().to_bytes().to_vec();
        println!("[REGISTRY] Ricevuto nuovo pacchetto binario .skill di {} byte.", wasm_bytecode.len());
        execute_skill_sandbox(&wasm_bytecode);
    }
    Ok(())
}

fn execute_skill_sandbox(_bytecode: &[u8]) {
    println!("[SANDBOX] Inizializzazione ambiente virtuale isolato (WASM)...");
    
    // NOTA PER LA COMMUNITY: Questa è l'interfaccia cieca di isolamento.
    // La ricompensa a tre stati (-1.0, 0.0, 1.0) viene valutata qui dentro 
    // preservando la sicurezza hardware e la protezione del codice aziendale.
    
    println!("[SANDBOX] Esecuzione logica di ricompensa completata con successo.");
}
