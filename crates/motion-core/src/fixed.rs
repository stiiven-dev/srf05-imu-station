//! Fixed-point (Q16.16) complementary filter and CORDIC atan2, for
//! benchmarking against the f32/libm implementation on a core with no FPU
//! and no hardware divide (Cortex-M0+). Restricted to inputs with x >= 0
//! (angles in ±90°) — sufficient for this project's ±30° bubble level,
//! not a general-purpose atan2 replacement.

/// Q16.16 fixed-point: 16 integer bits, 16 fractional bits, stored in i32.
pub type Q16 = i32;
const ONE: Q16 = 1 << 16;

/// atan(2^-i) for i = 0..=15, in Q16.16 degrees. Standard CORDIC table.
const ANGLE_TABLE: [Q16; 16] = [
    2_949_120, 1_740_977, 919_879, 466_945, 234_379, 117_304, 58_666, 29_335, 14_668, 7_334, 3_667,
    1_833, 917, 458, 229, 115,
];

/// atan2(y, x) in degrees (Q16.16), for x >= 0 only. Uses CORDIC vectoring
/// mode: no multiply, no divide — only shifts and adds.
pub fn cordic_atan2_deg(y: i32, x: i32) -> Q16 {
    debug_assert!(
        x >= 0,
        "cordic_atan2_deg: x must be non-negative (±90° range only)"
    );

    let mut x = x as i64;
    let mut y = y as i64;
    let mut angle: i64 = 0;

    for (i, &table_val) in ANGLE_TABLE.iter().enumerate() {
        let x_shift = x >> i;
        let y_shift = y >> i;
        if y > 0 {
            let new_x = x + y_shift;
            let new_y = y - x_shift;
            x = new_x;
            y = new_y;
            angle += table_val as i64;
        } else {
            let new_x = x - y_shift;
            let new_y = y + x_shift;
            x = new_x;
            y = new_y;
            angle -= table_val as i64;
        }
    }
    angle as Q16
}

/// Integer square root via bit-guessing (Newton's method variant) — no
/// division, no floating point.
pub fn isqrt(n: u32) -> u32 {
    if n == 0 {
        return 0;
    }
    let mut x = n;
    let mut y = x.div_ceil(2);
    while y < x {
        x = y;
        y = (x + n / x) / 2;
    }
    x
}

pub struct FixedComplementaryFilter {
    roll_q16: Q16, // degrees, Q16.16
    pitch_q16: Q16,
    alpha_q16: Q16, // e.g. 0.98 -> 64225
}

impl FixedComplementaryFilter {
    pub fn new(alpha: f32) -> Self {
        Self {
            roll_q16: 0,
            pitch_q16: 0,
            alpha_q16: (alpha * ONE as f32) as Q16,
        }
    }

    /// `gyro_raw`/`accel_raw` are (x, y, z) in raw sensor LSB — no unit
    /// conversion needed (see module docs). `dt_us` is the real interval
    /// since the last update, in microseconds.
    pub fn update(
        &mut self,
        dt_us: u32,
        gyro_raw: (i16, i16, i16),
        accel_raw: (i16, i16, i16),
    ) -> (Q16, Q16) {
        let (gx, gy, _gz) = gyro_raw;
        let (ax, ay, az) = accel_raw;

        // atan2 is scale-invariant: raw LSB in, degrees out, no g-conversion.
        let accel_roll = cordic_atan2_deg(ay as i32, az.max(1) as i32);
        let mag = isqrt((ay as i32 * ay as i32 + az as i32 * az as i32) as u32);
        let accel_pitch = cordic_atan2_deg(-(ax as i32), mag as i32);

        // gx (raw LSB) -> degrees/sec (Q16.16): gx / 131.0
        let gx_dps_q16 = ((gx as i64) << 16) / 131;
        let gy_dps_q16 = ((gy as i64) << 16) / 131;
        // dt in Q16.16 seconds
        let dt_q16 = ((dt_us as i64) << 16) / 1_000_000;

        let gyro_delta_roll = ((gx_dps_q16 * dt_q16) >> 16) as Q16;
        let gyro_delta_pitch = ((gy_dps_q16 * dt_q16) >> 16) as Q16;

        let one_minus_alpha = ONE - self.alpha_q16;

        self.roll_q16 = (((self.alpha_q16 as i64) * ((self.roll_q16 + gyro_delta_roll) as i64))
            >> 16) as Q16
            + (((one_minus_alpha as i64) * (accel_roll as i64)) >> 16) as Q16;
        self.pitch_q16 = (((self.alpha_q16 as i64) * ((self.pitch_q16 + gyro_delta_pitch) as i64))
            >> 16) as Q16
            + (((one_minus_alpha as i64) * (accel_pitch as i64)) >> 16) as Q16;

        (self.pitch_q16, self.roll_q16)
    }
}

/// Convert a Q16.16 value to f32, for comparing against the reference
/// implementation in tests and logs.
pub fn q16_to_f32(q: Q16) -> f32 {
    q as f32 / ONE as f32
}

#[cfg(test)]
mod fixed_tests {
    use super::*;

    fn assert_close_deg(actual: Q16, expected_deg: f32, tol_deg: f32) {
        let actual_deg = q16_to_f32(actual);
        assert!(
            (actual_deg - expected_deg).abs() < tol_deg,
            "expected ~{expected_deg}°, got {actual_deg}°"
        );
    }

    #[test]
    fn atan2_zero_is_zero() {
        assert_close_deg(cordic_atan2_deg(0, 100), 0.0, 0.5);
    }

    #[test]
    fn atan2_45_degrees() {
        assert_close_deg(cordic_atan2_deg(100, 100), 45.0, 0.5);
    }

    #[test]
    fn atan2_near_90_degrees() {
        assert_close_deg(cordic_atan2_deg(1000, 1), 90.0, 1.0);
    }

    #[test]
    fn atan2_negative_y() {
        assert_close_deg(cordic_atan2_deg(-100, 100), -45.0, 0.5);
    }

    #[test]
    fn isqrt_known_values() {
        assert_eq!(isqrt(0), 0);
        assert_eq!(isqrt(1), 1);
        assert_eq!(isqrt(16), 4);
        assert_eq!(isqrt(1_000_000), 1000);
        assert_eq!(isqrt(16384 * 16384), 16384);
    }

    #[test]
    fn fixed_filter_agrees_with_f32_reference_within_tolerance() {
        let mut fixed = FixedComplementaryFilter::new(0.98);
        let mut reference = crate::ComplementaryFilter::new(0.98);

        let samples: [((i16, i16, i16), (i16, i16, i16)); 3] = [
            ((5, -3, 2), (0, 0, 16384)),
            ((5, -3, 2), (0, 100, 16350)),
            ((5, -3, 2), (0, 500, 16000)),
        ];

        for (gyro, accel) in samples {
            let (fx_pitch, fx_roll) = fixed.update(10_000, gyro, accel);
            let att = reference.update(
                0.01,
                (
                    gyro.0 as f32 / 131.0,
                    gyro.1 as f32 / 131.0,
                    gyro.2 as f32 / 131.0,
                ),
                (
                    accel.0 as f32 / 16384.0,
                    accel.1 as f32 / 16384.0,
                    accel.2 as f32 / 16384.0,
                ),
            );
            assert!(
                (q16_to_f32(fx_roll) - att.roll_deg).abs() < 1.0,
                "roll diverged"
            );
            assert!(
                (q16_to_f32(fx_pitch) - att.pitch_deg).abs() < 1.0,
                "pitch diverged"
            );
        }
    }

    #[test]
    fn angle_table_matches_known_atan_values() {
        // atan(2^-i) in degrees, computed independently via libm, compared
        // against the hardcoded ANGLE_TABLE entries.
        for (i, &table_val) in ANGLE_TABLE.iter().enumerate() {
            let expected_deg = libm::atanf(1.0 / (1u32 << i) as f32).to_degrees();
            let table_deg = table_val as f32 / ONE as f32;
            assert!(
                (expected_deg - table_deg).abs() < 0.01,
                "ANGLE_TABLE[{i}]: expected ~{expected_deg}°, table has {table_deg}°"
            );
        }
    }
}
