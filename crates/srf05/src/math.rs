#![allow(clippy::manual_range_contains)]

/// A completed SRF05 reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reading {
    /// Distance to nearest reflective surface.
    Mm(u16),
    /// A valid echo returned, but implies a distance beyond the sensor's
    /// useful range (an echo held high for more than 30ms).
    OutOfRange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// No falling edge arrived within the timeout: check wiring/power.
    TimeOut,
}

const MAX_RANGE_MM: u64 = 4_000;

/// Convert an echo pulse width from microseconds to a distance.
///
/// Speed of sound = 343 m/s; the pulse covers a round trip, hence the division by 2000
/// (343 m/s = 343 mm/ms = 0.343 mm/µs, and we divide the round trip by 2).
pub fn pulse_width_to_mm(width_us: u32) -> Reading {
    let mm = (width_us.min(100_000) as u64) * 343 / 2000; //casting to u64 to avoid overflow
    if mm > MAX_RANGE_MM {
        Reading::OutOfRange
    } else {
        Reading::Mm(mm as u16)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn typical_pulse_converts_correctly() {
        // ~5831us round-trip ≈ 1000mm, per the SRF05 datasheet formula.
        assert_eq!(pulse_width_to_mm(5831), Reading::Mm(1000));
    }

    #[test]
    fn very_long_pulse_is_out_of_range() {
        assert_eq!(pulse_width_to_mm(30_000), Reading::OutOfRange);
    }

    #[test]
    fn zero_width_is_zero_distance() {
        assert_eq!(pulse_width_to_mm(0), Reading::Mm(0));
    }
}
