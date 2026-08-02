/// Dimensione massima fissa dei vettori di stato e di comando.
///
/// 32 giunti coprono con abbondanza qualunque robot (bracci, AMR, droni,
/// umanoidi) e garantiscono che `StateVector`/`CommandVector` siano array
/// statici pre-allocati: nessun `malloc` durante il loop di controllo a
/// 100 Hz, quindi zero frammentazione dell'heap.
pub const MAX_JOINTS: usize = 32;

/// Rappresentazione matematica standardizzata dei comandi diretti ai motori.
///
/// Vettore a dimensione **fissa** `[f32; 32]` + lunghezza attiva `len`: la RAM
/// occupata è identica dal primo secondo fino a 10 anni di utilizzo continuo.
#[derive(Debug, Clone, Copy)]
pub struct CommandVector {
    pub target_actuators: [f32; MAX_JOINTS], // Voltaggi o coppie per i giunti
    pub len: usize,
}

impl CommandVector {
    /// Vettore vuoto, tutto a zero. Nessuna allocazione.
    pub const fn new() -> Self {
        Self {
            target_actuators: [0.0; MAX_JOINTS],
            len: 0,
        }
    }

    /// Vettore di zeri lungo `joint_count`: usato da estop e Hard-Watchdog.
    pub fn zeros(joint_count: usize) -> Self {
        Self {
            target_actuators: [0.0; MAX_JOINTS],
            len: joint_count.min(MAX_JOINTS),
        }
    }

    /// Copia `values` nell'array statico (troncato a `MAX_JOINTS`).
    pub fn from_slice(values: &[f32]) -> Self {
        let mut v = Self::new();
        let n = values.len().min(MAX_JOINTS);
        v.target_actuators[..n].copy_from_slice(&values[..n]);
        v.len = n;
        v
    }

    /// Converte i byte estratti da Zenoh nuovamente in comandi f32 per il robot.
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut v = Self::new();
        let mut n = 0;
        for chunk in bytes.chunks_exact(4) {
            if n >= MAX_JOINTS {
                break;
            }
            v.target_actuators[n] = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            n += 1;
        }
        v.len = n;
        v
    }

    pub fn as_slice(&self) -> &[f32] {
        &self.target_actuators[..self.len]
    }

    pub fn as_mut_slice(&mut self) -> &mut [f32] {
        &mut self.target_actuators[..self.len]
    }

    /// Serializzazione binaria compatta **senza allocazione**: scrive i valori
    /// f32 LE in `out` e restituisce i byte scritti. Stesso formato usato dagli
    /// `StateVector`: un unico parser lato microcontrollore per telemetria e
    /// comandi. `out` deve avere capacità `MAX_JOINTS * 4` (128 byte).
    pub fn write_to(&self, out: &mut [u8]) -> usize {
        let n = self.len.min(out.len() / 4);
        let mut written = 0;
        for i in 0..n {
            let bytes = self.target_actuators[i].to_le_bytes();
            out[written..written + 4].copy_from_slice(&bytes);
            written += 4;
        }
        written
    }
}

impl Default for CommandVector {
    fn default() -> Self {
        Self::new()
    }
}
