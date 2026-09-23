#![cfg_attr(not(test), no_std)]

mod blocking;
mod capture;
mod math;

pub use blocking::{MeasureError, Srf05};
pub use capture::EdgeCapture;
pub use math::{Error, Reading, pulse_width_to_mm};
