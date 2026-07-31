use std::time::Duration;
use tokio::time::sleep;

pub struct TerminalUi {
    pub robot_name: String,
}

impl TerminalUi {
    pub fn new(name: &str) -> Self {
        Self {
            robot_name: name.to_string(),
        }
    }

    /// Avvia il loop di rendering grafico asincrono all'interno del terminale
    pub async fn start_render_loop(&self) {
        println!("[DIAGNOSTICS] Avvio interfaccia Terminal UI di SONNY...");
        
        // Simula un loop di monitoraggio dei widget (frequenza di aggiornamento 2Hz)
        loop {
            sleep(Duration::from_millis(500)).await;
            
            // Pulisce lo schermo del terminale per evitare l'effetto sfarfallio
            print!("{}[2J{}[H", 27 as char, 27 as char); 
            
            println!("====================================================");
            println!("      SONNY - OPERATING SYSTEM DASHBOARD   ");
            println!("====================================================");
            println!(" HARDWARE NODE:    {}", self.robot_name);
            println!(" CORE MEMORY BUS:  ONLINE (Latenza Zero)");
            println!(" NETWORK BUS:      ZENOH STREAMING [ACTIVE]");
            println!("----------------------------------------------------");
            println!(" BRAIN INTERFACE:  [HOOKED] -> Sistema Privato ");
            println!(" STATUS MONITOR:   In ascolto sui vettori di stato...");
            println!("====================================================");
        }
    }
}
