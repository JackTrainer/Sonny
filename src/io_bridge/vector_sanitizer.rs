use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};

/// Intercettore di sicurezza dei comandi destinati agli azionamenti.
///
/// La community o un produttore hardware esterno possono scrivere un
/// OpenHalConfig (o una Skill WASM) con dati sporchi: un NaN, un inversione
/// di segno, o un valore infinito generato da un sensore difettoso. Se un
/// comando così raggiungesse i motori reali, il robot compirebbe uno scatto
/// violento e si distruggerebbe.
///
/// Questo modulo scansiona il vettore di comando **prima** della spedizione
/// e blocca ogni anomalia: NaN/Infinity vengono sostituiti all'istante con
/// l'ultimo valore sicuro registrato nel tick precedente, poi il comando
/// viene limitato ai limiti fisici dichiarati nel JSON. Zero allocazioni:
/// lo storico vive in un array statico pre-allocato.
///
/// Esegue esclusivamente sul **Core 1** della CPU, dentro il thread dedicato
/// del loop di controllo a 100 Hz (`RealTimeControlLoop` in
/// `diagnostics::frequency_enforcer`): la sanificazione non condivide mai la
/// cache con il runtime Wasmtime del Core 2.
pub struct VectorSanitizer {
    last_safe: [f32; MAX_JOINTS],
}

impl VectorSanitizer {
    pub const fn new() -> Self {
        Self {
            last_safe: [0.0; MAX_JOINTS],
        }
    }

    /// Pulisce il vettore da anomalie matematiche radicali e lo clampa prima
    /// dell'invio all'HAL. Il comando viene modificato *in-place*:
    ///
    /// 1. NaN/Infinity → ripristino dell'ultimo valore sicuro del giunto;
    /// 2. valori fuori limite → taglio ai limiti fisici dichiarati nel JSON.
    ///
    /// I valori che superano entrambi i controlli diventano il nuovo punto di
    /// riferimento sicuro per i tick successivi.
    pub fn sanitize_and_clamp(&mut self, command: &mut CommandVector, limits: &[(f32, f32)]) {
        let n = command.len.min(MAX_JOINTS);
        for i in 0..n {
            let mut v = command.target_actuators[i];

            // 1. Protezione contro NaN/Infinity causati da calcoli errati di
            //    terze parti (config JSON sporca, sensore difettoso).
            if v.is_nan() || v.is_infinite() {
                v = self.last_safe[i];
                eprintln!(
                    "[WARN - HAL] Rilevato valore illegale (NaN/Inf) al giunto [{}]. Forzato ripristino sicuro.",
                    i
                );
            }

            // 2. Clamping rigido sui limiti fisici dichiarati nel JSON.
            if let Some((min, max)) = limits.get(i) {
                v = v.clamp(*min, *max);
            }

            command.target_actuators[i] = v;
            self.last_safe[i] = v;
        }
    }
}

impl Default for VectorSanitizer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nan_replaced_with_last_safe_value() {
        let mut sanitizer = VectorSanitizer::new();
        let limits = vec![(0.0, 1.0), (0.0, 1.0), (0.0, 1.0)];
        let mut cmd = CommandVector::from_slice(&[0.5, f32::NAN, 2.0]);
        sanitizer.sanitize_and_clamp(&mut cmd, &limits);
        assert_eq!(cmd.as_slice(), &[0.5, 0.0, 1.0][..]);
    }

    #[test]
    fn infinite_restores_previous_tick_value() {
        let mut sanitizer = VectorSanitizer::new();
        let limits = vec![(-1.0, 1.0)];

        let mut ok = CommandVector::from_slice(&[0.7]);
        sanitizer.sanitize_and_clamp(&mut ok, &limits);
        assert_eq!(ok.as_slice(), &[0.7][..]);

        let mut bad = CommandVector::from_slice(&[f32::INFINITY]);
        sanitizer.sanitize_and_clamp(&mut bad, &limits);
        assert_eq!(bad.as_slice(), &[0.7][..]);
    }

    #[test]
    fn newest_safe_value_is_the_reference() {
        let mut sanitizer = VectorSanitizer::new();
        let limits = vec![(-1.0, 1.0)];

        let mut a = CommandVector::from_slice(&[0.3]);
        sanitizer.sanitize_and_clamp(&mut a, &limits);

        let mut b = CommandVector::from_slice(&[-0.9]);
        sanitizer.sanitize_and_clamp(&mut b, &limits);

        let mut bad = CommandVector::from_slice(&[f32::NAN]);
        sanitizer.sanitize_and_clamp(&mut bad, &limits);
        assert_eq!(bad.as_slice(), &[-0.9][..]);
    }

    #[test]
    fn clamps_out_of_range_values() {
        let mut sanitizer = VectorSanitizer::new();
        let limits = vec![(-2.0, 2.0), (0.0, 1.0)];
        let mut cmd = CommandVector::from_slice(&[10.0, -5.0]);
        sanitizer.sanitize_and_clamp(&mut cmd, &limits);
        assert_eq!(cmd.as_slice(), &[2.0, 0.0][..]);
    }

    #[test]
    fn missing_limits_pass_through() {
        let mut sanitizer = VectorSanitizer::new();
        let mut cmd = CommandVector::from_slice(&[42.0, -42.0]);
        sanitizer.sanitize_and_clamp(&mut cmd, &[]);
        assert_eq!(cmd.as_slice(), &[42.0, -42.0][..]);
    }
}
