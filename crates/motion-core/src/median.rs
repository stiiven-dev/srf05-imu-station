//! Fixed-window median filter (no heap, no_std).

/// Sentinel for "no valid reading". It sorts above every real distance, so a
/// lone dropout is outvoted like any other outlier, while a majority of
/// dropouts makes the median itself `NO_READING`.
pub const NO_READING: u16 = u16::MAX;

/// Median of the last 'N' samples. 'N' must be odd.
#[derive(Clone, Copy)]
pub struct MedianFilter<const N: usize> {
    window: [u16; N],
    next: usize, // where the next value should be written; wraps around N
    len: usize,  // how many slots are available
}

impl<const N: usize> MedianFilter<N> {
    pub const fn new() -> Self {
        assert!(N % 2 == 1, "window size must be odd");
        Self {
            window: [0; N],
            next: 0,
            len: 0,
        }
    }

    /// Feed one sample. Returns None until the window is full, then
    /// the median of the most recent N samples.
    pub fn push(&mut self, sample: u16) -> Option<u16> {
        self.window[self.next] = sample; //overwrite the oldest value
        self.next = (self.next + 1) % N;
        if self.len < N {
            self.len += 1;
        }
        if self.len < N {
            return None;
        }
        Some(self.median())
    }

    /// Forget all history
    pub fn reset(&mut self) {
        self.next = 0;
        self.len = 0;
    }

    fn median(&self) -> u16 {
        let mut sorted = self.window;
        sorted.sort_unstable();
        sorted[N / 2]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed<const N: usize>(f: &mut MedianFilter<N>, xs: &[u16]) -> Option<u16> {
        let mut last = None;
        for &x in xs {
            last = f.push(x);
        }
        last
    }

    #[test]
    fn no_output_until_window_is_full() {
        let mut f = MedianFilter::<5>::new();
        for x in [10, 20, 30, 40] {
            assert_eq!(f.push(x), None);
        }
        assert!(f.push(50).is_some());
    }

    #[test]
    fn constant_input_passes_through() {
        let mut f = MedianFilter::<5>::new();
        assert_eq!(feed(&mut f, &[300; 5]), Some(300));
    }

    #[test]
    fn median_is_the_middle_value_regardless_of_order() {
        let mut f = MedianFilter::<5>::new();
        assert_eq!(feed(&mut f, &[5, 1, 4, 2, 3]), Some(3));
    }

    #[test]
    fn single_spike_is_rejected() {
        let mut f = MedianFilter::<5>::new();
        assert_eq!(feed(&mut f, &[500, 502, 4000, 499, 501]), Some(501));
    }

    #[test]
    fn single_dropout_is_rejected() {
        let mut f = MedianFilter::<5>::new();
        assert_eq!(feed(&mut f, &[500, NO_READING, 502, 499, 501]), Some(501));
    }

    #[test]
    fn majority_dropouts_report_no_reading() {
        let mut f = MedianFilter::<5>::new();
        let out = feed(&mut f, &[500, NO_READING, NO_READING, 499, NO_READING]);
        assert_eq!(out, Some(NO_READING));
    }

    #[test]
    fn step_change_appears_on_the_third_new_sample() {
        let mut f = MedianFilter::<5>::new();
        feed(&mut f, &[100; 5]);
        assert_eq!(f.push(200), Some(100));
        assert_eq!(f.push(200), Some(100));
        assert_eq!(f.push(200), Some(200)); // (N+1)/2 samples of latency
    }

    #[test]
    fn isolated_spikes_never_reach_the_output() {
        // True distance ~800 mm with a spike every 4th sample: any 5-sample
        // window holds at most 2 spikes, which is still fewer than a majority.
        let data: Vec<u16> = (0..40)
            .map(|i| {
                if i % 4 == 3 {
                    3000
                } else {
                    798 + (i % 5) as u16
                }
            })
            .collect();
        assert_eq!(data.iter().max(), Some(&3000)); // the raw data does have spikes
        let mut f = MedianFilter::<5>::new();
        for &x in &data {
            if let Some(y) = f.push(x) {
                assert!((798..=802).contains(&y), "spike leaked: {y}");
            }
        }
    }

    #[test]
    fn reset_forgets_history() {
        let mut f = MedianFilter::<5>::new();
        feed(&mut f, &[100; 5]);
        f.reset();
        assert_eq!(f.push(100), None);
    }

    #[test]
    #[should_panic]
    fn even_window_is_rejected() {
        let _ = MedianFilter::<4>::new();
    }
}
