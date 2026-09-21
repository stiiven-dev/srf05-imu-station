#![cfg_attr(not(test), no_std)]

mod median;

pub use median::{MedianFilter, NO_READING};
