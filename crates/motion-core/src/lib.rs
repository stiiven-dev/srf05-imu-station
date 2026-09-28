#![cfg_attr(not(test), no_std)]

mod calibration;
mod filter;
mod fixed;
mod median;

pub use calibration::{Calibration, Calibrator, RawSample3};
pub use filter::{Attitude, ComplementaryFilter};
pub use fixed::{FixedComplementaryFilter, Q16, cordic_atan2_deg, isqrt, q16_to_f32};
pub use median::{MedianFilter, NO_READING};
