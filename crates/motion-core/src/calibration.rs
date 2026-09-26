//! Gyro bias / accelerometer offset calibration: average N stationary
//! samples, derive the correction to apply to every future reading.

/// One raw 3-axis sample, straight from the sensor's registers.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RawSample3 {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Gyro bias (subtract from every future gyro reading) and accelerometer
/// offset (subtract from every future accel reading) derived from a
/// stationary calibration window.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Calibration {
    pub gyro_bias: RawSample3,
    pub accel_offset: RawSample3,
}

impl Calibration {
    /// Remove this calibration's fixed bias from a raw gyro sample.
    pub fn correct_gyro(&self, raw: RawSample3) -> RawSample3 {
        RawSample3 {
            x: sub_sat(raw.x, self.gyro_bias.x),
            y: sub_sat(raw.y, self.gyro_bias.y),
            z: sub_sat(raw.z, self.gyro_bias.z),
        }
    }

    /// Remove this calibration's fixed offset from a raw accelerometer
    /// sample. A stationary, level board should read close to its expected
    /// resting value (see `EXPECTED_ACCEL_Z_AT_REST`) afterward.
    pub fn correct_accel(&self, raw: RawSample3) -> RawSample3 {
        RawSample3 {
            x: sub_sat(raw.x, self.accel_offset.x),
            y: sub_sat(raw.y, self.accel_offset.y),
            z: sub_sat(raw.z, self.accel_offset.z),
        }
    }
}

/// The board's Z axis reads this many LSB at rest, assuming the ±2g range
/// (16384 LSB/g) and the board mounted flat with Z pointing up. If your
/// wiring has Z pointing down at rest, flip the sign here once confirmed
/// on hardware.
const EXPECTED_ACCEL_Z_AT_REST: i32 = 16384;

/// Accumulates stationary samples and produces a [`Calibration`] once
/// enough have been collected. `N` is the number of samples to average;
/// more reduces noise in the estimate but takes longer to hold still for.
pub struct Calibrator<const N: usize> {
    count: usize,
    gyro_sum: [i32; 3],
    accel_sum: [i32; 3],
}

impl<const N: usize> Calibrator<N> {
    pub const fn new() -> Self {
        Self {
            count: 0,
            gyro_sum: [0; 3],
            accel_sum: [0; 3],
        }
    }

    /// Feed one stationary sample. Returns `Some(Calibration)` once `N`
    /// samples have been collected; call sites should stop calling `push`
    /// after that (or construct a new `Calibrator` to start over).
    pub fn push(&mut self, gyro: RawSample3, accel: RawSample3) -> Option<Calibration> {
        self.gyro_sum[0] += i32::from(gyro.x);
        self.gyro_sum[1] += i32::from(gyro.y);
        self.gyro_sum[2] += i32::from(gyro.z);
        self.accel_sum[0] += i32::from(accel.x);
        self.accel_sum[1] += i32::from(accel.y);
        self.accel_sum[2] += i32::from(accel.z);
        self.count += 1;
        if self.count < N {
            return None;
        }

        let n = N as i32;
        let gyro_bias = RawSample3 {
            x: (self.gyro_sum[0] / n) as i16,
            y: (self.gyro_sum[1] / n) as i16,
            z: (self.gyro_sum[2] / n) as i16,
        };
        let accel_offset = RawSample3 {
            x: (self.accel_sum[0] / n) as i16,
            y: (self.accel_sum[1] / n) as i16,
            z: ((self.accel_sum[2] / n) - EXPECTED_ACCEL_Z_AT_REST) as i16,
        };

        Some(Calibration {
            gyro_bias,
            accel_offset,
        })
    }

    pub fn samples_collected(&self) -> usize {
        self.count
    }
}

impl<const N: usize> Default for Calibrator<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Subtract with clamping instead of panicking/wrapping on overflow — a
/// pathological reading shouldn't crash the correction step.
fn sub_sat(a: i16, b: i16) -> i16 {
    (i32::from(a) - i32::from(b)).clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn perfect_sensor_yields_zero_bias() {
        let mut cal = Calibrator::<10>::new();
        let mut result = None;
        for _ in 0..10 {
            result = cal.push(
                RawSample3 { x: 0, y: 0, z: 0 },
                RawSample3 {
                    x: 0,
                    y: 0,
                    z: EXPECTED_ACCEL_Z_AT_REST as i16,
                },
            );
        }
        assert_eq!(result, Some(Calibration::default()));
    }

    #[test]
    fn constant_gyro_offset_is_captured_as_bias() {
        let mut cal = Calibrator::<5>::new();
        let mut result = None;
        for _ in 0..5 {
            result = cal.push(
                RawSample3 { x: 12, y: -8, z: 3 },
                RawSample3 {
                    x: 0,
                    y: 0,
                    z: EXPECTED_ACCEL_Z_AT_REST as i16,
                },
            );
        }
        let cal = result.unwrap();
        assert_eq!(cal.gyro_bias, RawSample3 { x: 12, y: -8, z: 3 });
    }

    #[test]
    fn noisy_samples_average_out() {
        // Alternating +2/-2 around a true bias of 10 should average to 10.
        let mut cal = Calibrator::<4>::new();
        let mut result = None;
        for x in [12, 8, 12, 8] {
            result = cal.push(
                RawSample3 { x, y: 0, z: 0 },
                RawSample3 {
                    x: 0,
                    y: 0,
                    z: EXPECTED_ACCEL_Z_AT_REST as i16,
                },
            );
        }
        assert_eq!(result.unwrap().gyro_bias.x, 10);
    }

    #[test]
    fn no_result_before_n_samples() {
        let mut cal = Calibrator::<5>::new();
        for _ in 0..4 {
            assert!(
                cal.push(RawSample3::default(), RawSample3::default())
                    .is_none()
            );
        }
    }

    #[test]
    fn accel_offset_measured_relative_to_expected_gravity() {
        let mut cal = Calibrator::<3>::new();
        let mut result = None;
        for _ in 0..3 {
            // Z reads 100 LSB high of the expected resting value.
            result = cal.push(
                RawSample3::default(),
                RawSample3 {
                    x: 0,
                    y: 0,
                    z: (EXPECTED_ACCEL_Z_AT_REST + 100) as i16,
                },
            );
        }
        assert_eq!(result.unwrap().accel_offset.z, 100);
    }

    #[test]
    fn correcting_with_its_own_bias_cancels_it_out() {
        let cal = Calibration {
            gyro_bias: RawSample3 { x: 12, y: -8, z: 3 },
            accel_offset: RawSample3 {
                x: 5,
                y: -5,
                z: 100,
            },
        };
        let raw_gyro = RawSample3 { x: 12, y: -8, z: 3 };
        assert_eq!(cal.correct_gyro(raw_gyro), RawSample3 { x: 0, y: 0, z: 0 });
    }

    #[test]
    fn correction_clamps_instead_of_overflowing() {
        let cal = Calibration {
            gyro_bias: RawSample3 {
                x: -20000,
                y: 0,
                z: 0,
            },
            accel_offset: RawSample3::default(),
        };
        let raw = RawSample3 {
            x: 20000,
            y: 0,
            z: 0,
        };
        // 20000 - (-20000) = 40000, which overflows i16::MAX (32767)
        assert_eq!(cal.correct_gyro(raw).x, i16::MAX);
    }
}
