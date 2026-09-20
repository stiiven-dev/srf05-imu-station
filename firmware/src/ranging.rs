#[derive(Clone, Copy)]
pub struct EchoCapture {
    rise_us: Option<u64>,
    width_us: Option<u32>,
}

impl EchoCapture {
    pub const fn new() -> Self {
        Self {
            rise_us: None,
            width_us: None,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn on_rise(&mut self, now_us: u64) {
        self.rise_us = Some(now_us);
    }

    pub fn on_fall(&mut self, now_us: u64) {
        // A falling edge with no prior rising edge is noise , ignore it.
        if let Some(rise) = self.rise_us.take() {
            let w = now_us.saturating_sub(rise);
            self.width_us = Some(u32::try_from(w).unwrap_or(u32::MAX));
        }
    }

    pub fn take_result(&mut self) -> Option<u32> {
        self.width_us.take()
    }
}

#[derive(Clone, Copy)]
pub enum Reading {
    Mm(u16),
    OutOfRange,
    NoEcho,
}

const MAX_RANGE_MM: u64 = 4000;

pub fn classify(width_us: Option<u32>) -> Reading {
    match width_us {
        None => Reading::NoEcho,
        Some(w) => {
            let mm = (w.min(100_000) as u64) * 343 / 2000;
            if mm > MAX_RANGE_MM {
                Reading::OutOfRange
            } else {
                Reading::Mm(mm as u16)
            }
        }
    }
}
