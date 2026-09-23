use crate::math::{Error, Reading, pulse_width_to_mm};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};

/// Blocking SRF05 driver. Portable to any `embedded-hal` 1.0 implementation;
/// no interrupts, no timer peripheral required — only two GPIO pins and a
/// delay. Trades CPU time (busy-polls during the echo, up to ~30ms worst
/// case) for zero platform-specific setup.
pub struct Srf05<TRIG, ECHO, DELAY> {
    trig: TRIG,
    echo: ECHO,
    delay: DELAY,
}

#[derive(Debug)]
pub enum MeasureError<E> {
    /// The underlying GPIO operation failed.
    Pin(E),
    /// No echo, or it never returned to low: see [`Error`].
    Reading(Error),
}

/// Longest an SRF05 will hold ECHO high with no target in range.
const TIMEOUT_US: u32 = 35_000;
/// Polling granularity: coarser = less CPU-accurate, finer = more delay calls.
const POLL_STEP_US: u32 = 5;

impl<TRIG, ECHO, DELAY, E> Srf05<TRIG, ECHO, DELAY>
where
    TRIG: OutputPin<Error = E>,
    ECHO: InputPin<Error = E>,
    DELAY: DelayNs,
{
    pub fn new(trig: TRIG, echo: ECHO, delay: DELAY) -> Self {
        Self { trig, echo, delay }
    }

    /// Trigger a ping and block until a distance is measured or the
    /// sensor's own ~30ms no-target timeout elapses.
    pub fn measure(&mut self) -> Result<Reading, MeasureError<E>> {
        self.trig.set_high().map_err(MeasureError::Pin)?;
        self.delay.delay_us(10);
        self.trig.set_low().map_err(MeasureError::Pin)?;

        self.wait_for_level(true)?; // rising edge: echo pulse starts
        let width_us = self.time_high()?;
        Ok(pulse_width_to_mm(width_us))
    }

    fn wait_for_level(&mut self, high: bool) -> Result<(), MeasureError<E>> {
        let mut elapsed = 0u32;
        loop {
            let level = self.echo.is_high().map_err(MeasureError::Pin)?;
            if level == high {
                return Ok(());
            }
            if elapsed >= TIMEOUT_US {
                return Err(MeasureError::Reading(Error::TimeOut));
            }
            self.delay.delay_us(POLL_STEP_US);
            elapsed += POLL_STEP_US;
        }
    }
    fn time_high(&mut self) -> Result<u32, MeasureError<E>> {
        let mut elapsed = 0u32;
        while self.echo.is_high().map_err(MeasureError::Pin)? {
            if elapsed >= TIMEOUT_US {
                return Err(MeasureError::Reading(Error::TimeOut));
            }
            self.delay.delay_us(POLL_STEP_US);
            elapsed += POLL_STEP_US;
        }
        Ok(elapsed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, State as S, Transaction as T};

    #[test]
    fn happy_path_measures_a_distance() {
        let trig_expectations = [T::set(S::High), T::set(S::Low)];
        // ~1160us high ≈ 200mm: is_high() polled every 5us, alternating
        // High while the pulse is up, then Low once it ends.
        let mut echo_expectations = vec![T::get(S::Low)]; // not-yet-risen check
        echo_expectations.push(T::get(S::High)); // rising edge seen
        for _ in 0..232 {
            echo_expectations.push(T::get(S::High)); // pulse still up
        }
        echo_expectations.push(T::get(S::Low)); // falling edge seen

        let trig = PinMock::new(&trig_expectations);
        let echo = PinMock::new(&echo_expectations);
        let mut drv = Srf05::new(trig, echo, NoopDelay);

        let reading = drv.measure().unwrap();
        assert_eq!(reading, Reading::Mm(198)); // 232*5us=1160us of high time

        drv.trig.done();
        drv.echo.done(); // fails the test if any expectation was unused
    }

    #[test]
    fn no_rising_edge_times_out() {
        let mut trig = PinMock::new(&[T::set(S::High), T::set(S::Low)]);
        // ECHO never goes high: every poll during wait_for_level returns Low.
        let echo_expectations: Vec<_> = (0..=(35_000 / 5)).map(|_| T::get(S::Low)).collect();
        let mut echo = PinMock::new(&echo_expectations);
        let mut drv = Srf05::new(trig.clone(), echo.clone(), NoopDelay);

        let err = drv.measure().unwrap_err();
        assert!(matches!(err, MeasureError::Reading(Error::TimeOut)));
        echo.done();
        trig.done();
    }
}
