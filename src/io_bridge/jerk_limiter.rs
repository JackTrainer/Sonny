use crate::io_bridge::command_vector::{CommandVector, MAX_JOINTS};

/// Mechanical shock limiter: limits the **third derivative** (jerk) of the
/// command curve.
///
/// Even if a computed trajectory stays within the geometric limits (e.g.
/// the gripper moves from X=0 to X=10), if the displacement happens too
/// fast in a single tick, the sudden acceleration lashes the joints
/// mechanically, wearing them out or breaking the held object.
///
/// This module applies a third-order saturation chain on every
/// joint: if the acceleration variation between frame N and frame N-1
/// exceeds the machine's physical tolerance (`max_jerk`), the curve is
/// mathematically smoothed and slowed before dispatch to the HAL.
///
/// Everything lives in pre-allocated static arrays: no allocation during the
/// control loop.
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
    /// Creates the limiter with sampling period `dt` (seconds). The
    /// limits start at `+∞`: until configured with
    /// [`JerkLimiter::set_limits`], command pass-through is transparent.
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

    /// Configures the physical tolerances per joint, in rad/s, rad/s² and rad/s³.
    /// The slices may be shorter than `MAX_JOINTS`: the missing joints
    /// remain unlimited (transparent pass-through).
    pub fn set_limits(&mut self, max_vel: &[f32], max_acc: &[f32], max_jerk: &[f32]) {
        copy_limit(&mut self.max_vel, max_vel);
        copy_limit(&mut self.max_acc, max_acc);
        copy_limit(&mut self.max_jerk, max_jerk);
    }

    /// Smoothes the command curve *in-place*.
    ///
    /// For each joint: the velocity is limited to `max_vel`, the acceleration to
    /// `max_acc` and the **acceleration variation between frames N and N-1** (i.e.
    /// the jerk) to `max_jerk`. The result is re-clamped to the geometric
    /// `limits` as a final safety net.
    pub fn limit(&mut self, command: &mut CommandVector, limits: &[(f32, f32)]) {
        let n = command.len.min(MAX_JOINTS);
        let dt = self.dt;
        for i in 0..n {
            let target = command.target_actuators[i];
            let vmax = self.max_vel[i];
            let amax = self.max_acc[i];
            let jmax = self.max_jerk[i];

            let dist = target - self.prev_pos[i];

            // Approach time constant: large enough for the
            // acceleration and jerk limits to track the demand
            // without overshoot. With no limits (`+∞`) it equals `dt` and the
            // demand is transparent (the target is reached in one tick).
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

            // Desired velocity to approach the target, limited by the
            // maximum velocity and the available braking distance: the
            // demand can never exceed the velocity that allows
            // stopping within the target (no overshoot).
            let mut v_des = dist / tau;
            let mut v_lim = vmax;
            if amax.is_finite() {
                let v_stop = (2.0 * amax * dist.abs()).sqrt();
                v_lim = v_lim.min(v_stop);
            }
            if v_lim.is_finite() {
                v_des = v_des.clamp(-v_lim, v_lim);
            }

            // Desired acceleration to reach the velocity.
            let mut a_des = (v_des - self.prev_vel[i]) / dt;
            if amax.is_finite() {
                a_des = a_des.clamp(-amax, amax);
            }

            // Jerk limit: the acceleration variation between frame N
            // and frame N-1 cannot exceed jmax * dt.
            let mut a = if jmax.is_finite() {
                let a_step = jmax * dt;
                (a_des - self.prev_acc[i]).clamp(-a_step, a_step) + self.prev_acc[i]
            } else {
                a_des
            };
            if amax.is_finite() {
                a = a.clamp(-amax, amax);
            }

            // Numerical integration: smoothed velocity and position.
            let mut v = self.prev_vel[i] + a * dt;
            if vmax.is_finite() {
                v = v.clamp(-vmax, vmax);
            }
            let mut q = self.prev_pos[i] + v * dt;

            // Final geometric clamp: the position never leaves the limits.
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
        assert!(max_jerk <= 100.0 * 1.05, "jerk {:.3} exceeds the limit", max_jerk);
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
