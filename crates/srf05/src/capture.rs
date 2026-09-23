use crate::math::{Error, Reading, pulse_width_to_mm};

/// Accumulates a rising/falling edge pair into a distance reading.
///
/// Hardware-free: feed it timestamps from *any* clock, however you obtain
/// them (a hardware timer ISR, `embedded-hal-async::Wait`, anything with a
/// monotonically increasing microsecond counter). Useful for building your
/// own interrupt-driven capture without depending on this crate's blocking
/// or async drivers.
#[derive(Debug, Clone, Copy, Default)]
pub struct EdgeCapture {
    rise_us: Option<u64>,
    width_us: Option<u32>,
}

impl EdgeCapture {
    pub const fn new() -> Self {
        Self {
            rise_us: None,
            width_us: None,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Call from your rising-edge interrupt/event, with the current time.
    pub fn on_rise(&mut self, now_us: u64) {
        self.rise_us = Some(now_us);
    }

    /// Call from your falling-edge interrupt/event, with the current time.
    /// A falling edge with no prior rising edge is treated as noise.
    pub fn on_fall(&mut self, now_us: u64) {
        if let Some(rise) = self.rise_us.take() {
            let w = now_us.saturating_sub(rise);
            self.width_us = Some(u32::try_from(w).unwrap_or(u32::MAX));
        }
    }

    /// Take the completed reading, if the pulse has finished.
    pub fn take_reading(&mut self) -> Result<Reading, Error> {
        match self.width_us.take() {
            Some(w) => Ok(pulse_width_to_mm(w)),
            None => Err(Error::TimeOut),
        }
    }
}
