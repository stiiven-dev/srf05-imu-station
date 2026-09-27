//! Complementary filter: fuses gyro (smooth, drifts) and accel (noisy,
//! absolute) into a stable pitch/roll estimate.

use crate::RawSample3;

/// Gyro sensitivity at ±250°/s (see MPU-6050 datasheet).
const GYRO_LSB_PER_DPS: f32 = 131.0;
/// Accel sensitivity at ±2g.
const ACCEL_LSB_PER_G: f32 = 16384.0;

impl RawSample3 {
    pub fn gyro_to_dps(self) -> (f32, f32, f32) {
        (
            self.x as f32 / GYRO_LSB_PER_DPS,
            self.y as f32 / GYRO_LSB_PER_DPS,
            self.z as f32 / GYRO_LSB_PER_DPS,
        )
    }

    pub fn accel_to_g(self) -> (f32, f32, f32) {
        (
            self.x as f32 / ACCEL_LSB_PER_G,
            self.y as f32 / ACCEL_LSB_PER_G,
            self.z as f32 / ACCEL_LSB_PER_G,
        )
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Attitude {
    pub pitch_deg: f32,
    pub roll_deg: f32,
}

pub struct ComplementaryFilter {
    attitude: Attitude,
    alpha: f32,
}

impl ComplementaryFilter {
    pub const fn new(alpha: f32) -> Self {
        Self {
            attitude: Attitude {
                pitch_deg: 0.0,
                roll_deg: 0.0,
            },
            alpha,
        }
    }

    /// `gyro_dps`/`accel_g` are (x, y, z). `dt_s` is the time since the
    /// last update, in seconds.
    pub fn update(
        &mut self,
        dt_s: f32,
        gyro_dps: (f32, f32, f32),
        accel_g: (f32, f32, f32),
    ) -> Attitude {
        let (gx, gy, _gz) = gyro_dps;
        let (ax, ay, az) = accel_g;

        let accel_roll = libm::atan2f(ay, az).to_degrees();
        let accel_pitch = libm::atan2f(-ax, libm::sqrtf(ay * ay + az * az)).to_degrees();

        self.attitude.roll_deg =
            self.alpha * (self.attitude.roll_deg + gx * dt_s) + (1.0 - self.alpha) * accel_roll;
        self.attitude.pitch_deg =
            self.alpha * (self.attitude.pitch_deg + gy * dt_s) + (1.0 - self.alpha) * accel_pitch;

        self.attitude
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pure_gyro_alpha_one_just_integrates() {
        let mut f = ComplementaryFilter::new(1.0);
        let out = f.update(0.1, (10.0, 0.0, 0.0), (0.0, 0.0, 1.0));
        assert!((out.roll_deg - 1.0).abs() < 1e-4); // 10 dps * 0.1s
    }

    #[test]
    fn pure_accel_alpha_zero_reads_level() {
        let mut f = ComplementaryFilter::new(0.0);
        let out = f.update(0.1, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0));
        assert!(out.roll_deg.abs() < 1e-3);
        assert!(out.pitch_deg.abs() < 1e-3);
    }

    #[test]
    fn pure_accel_alpha_zero_reads_90_degree_roll() {
        let mut f = ComplementaryFilter::new(0.0);
        let out = f.update(0.1, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0));
        assert!((out.roll_deg - 90.0).abs() < 1e-2);
        assert!(out.pitch_deg.abs() < 1e-3); // unaffected by a pure-roll tilt
    }

    #[test]
    fn pure_accel_alpha_zero_reads_90_degree_pitch() {
        let mut f = ComplementaryFilter::new(0.0);
        let out = f.update(0.1, (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0));
        assert!((out.pitch_deg - 90.0).abs() < 1e-2);
    }
}
