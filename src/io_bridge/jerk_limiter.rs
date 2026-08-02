use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};

/// Limitatore di shock meccanico: limita la **derivata terza** (jerk) della
/// curva di comando.
///
/// Anche se una traiettoria calcolata resta dentro i limiti geometrici (es.
/// la pinza si sposta da X=0 a X=10), se lo spostamento avviene troppo
/// velocemente in un singolo tick, l'accelerazione improvvisa dà una frustata
/// meccanica ai giunti, usurandoli o spezzando l'oggetto preso.
///
/// Questo modulo applica una catena di saturazione del terzo ordine su ogni
/// giunto: se la variazione di accelerazione tra il frame N e il frame N-1
/// supera la tolleranza fisica della macchina (`max_jerk`), la curva viene
/// smussata e rallentata matematicamente prima della spedizione all'HAL.
///
/// Tutto vive in array statici pre-allocati: nessuna allocazione durante il
/// loop di controllo.
pub struct JerkLimiter {
    dt: f32,
    prev_pos: [f32; MAX_JOINTS],
    prev_vel: [f32; MAX_JOINTS],
    prev_acc: [f32; MAX_JOINTS],
    max_vel: [f32; MAX_JOINTS],
    max_acc: [f32; MAX_JOINTS],
    max_jerk: [f32; MAX_JOINTS],
}

impl JerkLimiter {
    /// Crea il limitatore con periodo di campionamento `dt` (secondi). I
    /// limiti partono da `+∞`: finché non vengono configurati con
    /// [`JerkLimiter::set_limits`], il passaggio dei comandi è trasparente.
    pub const fn new(dt: f32) -> Self {
        let dt = if dt > 0.0 { dt } else { 0.01 };
        Self {
            dt,
            prev_pos: [0.0; MAX_JOINTS],
            prev_vel: [0.0; MAX_JOINTS],
            prev_acc: [0.0; MAX_JOINTS],
            max_vel: [f32::INFINITY; MAX_JOINTS],
            max_acc: [f32::INFINITY; MAX_JOINTS],
            max_jerk: [f32::INFINITY; MAX_JOINTS],
        }
    }

    /// Configura le tolleranze fisiche per giunto, in rad/s, rad/s² e rad/s³.
    /// Le fette possono essere più corte di `MAX_JOINTS`: i giunti mancanti
    /// restano senza limite (passaggio trasparente).
    pub fn set_limits(&mut self, max_vel: &[f32], max_acc: &[f32], max_jerk: &[f32]) {
        copy_limit(&mut self.max_vel, max_vel);
        copy_limit(&mut self.max_acc, max_acc);
        copy_limit(&mut self.max_jerk, max_jerk);
    }

    /// Smussa la curva di comando *in-place*.
    ///
    /// Per ogni giunto: la velocità è limitata a `max_vel`, l'accelerazione a
    /// `max_acc` e la **variazione di accelerazione tra frame N e N-1** (cioè
    /// il jerk) a `max_jerk`. Il risultato viene re-clampato ai limiti
    /// geometrici `limits` come rete di sicurezza finale.
    pub fn limit(&mut self, command: &mut CommandVector, limits: &[(f32, f32)]) {
        let n = command.len.min(MAX_JOINTS);
        let dt = self.dt;
        for i in 0..n {
            let target = command.target_actuators[i];
            let vmax = self.max_vel[i];
            let amax = self.max_acc[i];
            let jmax = self.max_jerk[i];

            let dist = target - self.prev_pos[i];

            // Costante di tempo dell'avvicinamento: abbastanza grande perché i
            // limiti di accelerazione e jerk riescano a inseguire la richiesta
            // senza overshoot. Con limiti assenti (`+∞`) vale `dt` e la
            // richiesta è trasparente (il target viene raggiunto in un tick).
            let tau = {
                let from_vel = if vmax.is_finite() && amax.is_finite() {
                    vmax / amax
                } else {
                    0.0
                };
                let from_jerk = if amax.is_finite() && jmax.is_finite() {
                    amax / jmax
                } else {
                    0.0
                };
                (from_vel + from_jerk).max(dt)
            };

            // Velocità desiderata per avvicinarsi al target, limitata dalla
            // velocità massima e dalla distanza di frenata disponibile: la
            // richiesta non può mai eccedere la velocità che consente di
            // fermarsi entro il target (niente overshoot).
            let mut v_des = dist / tau;
            let mut v_lim = vmax;
            if amax.is_finite() {
                let v_stop = (2.0 * amax * dist.abs()).sqrt();
                v_lim = v_lim.min(v_stop);
            }
            if v_lim.is_finite() {
                v_des = v_des.clamp(-v_lim, v_lim);
            }

            // Accelerazione desiderata per portarsi alla velocità.
            let mut a_des = (v_des - self.prev_vel[i]) / dt;
            if amax.is_finite() {
                a_des = a_des.clamp(-amax, amax);
            }

            // Limite di jerk: la variazione di accelerazione tra il frame N
            // e il frame N-1 non può superare jmax * dt.
            let mut a = if jmax.is_finite() {
                let a_step = jmax * dt;
                (a_des - self.prev_acc[i]).clamp(-a_step, a_step) + self.prev_acc[i]
            } else {
                a_des
            };
            if amax.is_finite() {
                a = a.clamp(-amax, amax);
            }

            // Integrazione numerica: velocità e posizione smussate.
            let mut v = self.prev_vel[i] + a * dt;
            if vmax.is_finite() {
                v = v.clamp(-vmax, vmax);
            }
            let mut q = self.prev_pos[i] + v * dt;

            // Clamp geometrico finale: la posizione non esce mai dai limiti.
            if let Some((min, max)) = limits.get(i) {
                q = q.clamp(*min, *max);
            }

            command.target_actuators[i] = q;
            self.prev_pos[i] = q;
            self.prev_vel[i] = v;
            self.prev_acc[i] = a;
        }
    }
}

impl Default for JerkLimiter {
    fn default() -> Self {
        Self::new(0.01)
    }
}

fn copy_limit(dst: &mut [f32; MAX_JOINTS], src: &[f32]) {
    let n = src.len().min(MAX_JOINTS);
    dst[..n].copy_from_slice(&src[..n]);
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.01;

    #[test]
    fn passes_through_without_limits() {
        let mut lim = JerkLimiter::new(DT);
        let mut cmd = CommandVector::from_slice(&[0.7, -0.2]);
        lim.limit(&mut cmd, &[]);
        assert!((cmd.target_actuators[0] - 0.7).abs() < 1e-4);
        assert!((cmd.target_actuators[1] + 0.2).abs() < 1e-4);
    }

    #[test]
    fn geometric_limits_are_respected() {
        let mut lim = JerkLimiter::new(DT);
        let mut cmd = CommandVector::from_slice(&[5.0]);
        lim.limit(&mut cmd, &[(-1.0, 1.0)]);
        assert!((cmd.target_actuators[0] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn velocity_is_bounded() {
        let mut lim = JerkLimiter::new(DT);
        lim.set_limits(&[1.0], &[], &[]);

        let mut cmd = CommandVector::from_slice(&[1000.0]);
        lim.limit(&mut cmd, &[]);
        assert!((cmd.target_actuators[0] - 1.0 * DT).abs() < 1e-6);

        let mut cmd = CommandVector::from_slice(&[1000.0]);
        lim.limit(&mut cmd, &[]);
        assert!((cmd.target_actuators[0] - 2.0 * DT).abs() < 1e-6);
    }

    #[test]
    fn acceleration_is_bounded() {
        let mut lim = JerkLimiter::new(DT);
        lim.set_limits(&[], &[5.0], &[]);

        let mut cmd = CommandVector::from_slice(&[1000.0]);
        lim.limit(&mut cmd, &[]);
        let q1 = cmd.target_actuators[0];
        assert!((q1 - 5.0 * DT * DT).abs() < 1e-8);

        let mut cmd = CommandVector::from_slice(&[1000.0]);
        lim.limit(&mut cmd, &[]);
        let q2 = cmd.target_actuators[0];
        let implied_a = (q2 - 2.0 * q1) / (DT * DT);
        assert!((implied_a - 5.0).abs() < 1e-3);
    }

    #[test]
    fn jerk_is_bounded() {
        let mut lim = JerkLimiter::new(DT);
        lim.set_limits(&[], &[100.0], &[100.0]);

        let mut pos = Vec::with_capacity(200);
        let mut cmd = CommandVector::from_slice(&[1.0]);
        for _ in 0..200 {
            cmd.target_actuators[0] = 1.0;
            lim.limit(&mut cmd, &[]);
            pos.push(cmd.target_actuators[0]);
        }

        let mut max_jerk = 0.0f32;
        for k in 3..pos.len() {
            let a_n = (pos[k] - 2.0 * pos[k - 1] + pos[k - 2]) / (DT * DT);
            let a_p = (pos[k - 1] - 2.0 * pos[k - 2] + pos[k - 3]) / (DT * DT);
            let jerk = (a_n - a_p).abs() / DT;
            max_jerk = max_jerk.max(jerk);
        }
        assert!(max_jerk <= 100.0 * 1.05, "jerk {:.3} supera il limite", max_jerk);
    }

    #[test]
    fn step_response_does_not_overshoot() {
        let mut lim = JerkLimiter::new(DT);
        lim.set_limits(&[2.0], &[10.0], &[100.0]);

        let mut max_pos = 0.0f32;
        let mut cmd = CommandVector::from_slice(&[1.0]);
        for _ in 0..2000 {
            cmd.target_actuators[0] = 1.0;
            lim.limit(&mut cmd, &[]);
            max_pos = max_pos.max(cmd.target_actuators[0]);
        }
        assert!(max_pos <= 1.0 + 1e-3, "overshoot {:.4}", max_pos);
    }

    #[test]
    fn converges_to_target_smoothly() {
        let mut lim = JerkLimiter::new(DT);
        lim.set_limits(&[2.0], &[10.0], &[100.0]);

        let mut cmd = CommandVector::from_slice(&[1.0]);
        for _ in 0..500 {
            cmd.target_actuators[0] = 1.0;
            lim.limit(&mut cmd, &[]);
        }
        assert!((cmd.target_actuators[0] - 1.0).abs() < 1e-3);
    }
}
