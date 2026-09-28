# Timing characterization

Measured on-device using the RP2040's 1µs hardware timer (RTIC's `Mono`
monotonic) — no external probe or oscilloscope was used. These numbers
reflect software-observed timing only: they capture RTIC scheduling
precision and the MCU's view of its own GPIO commands, not true electrical
edge timing on the wire.

## Method

Instrumented `ranger` to log, once every 10 cycles:

- `trig_pulse_us` — measured duration of the 10µs TRIG pulse, CPU-observed
- `wake_jitter_us` — delay between the scheduled wake time and actual
  resumption after `Mono::delay_until`
- `period_us` — actual wall-clock time between successive cycle starts
  (nominal: 60,000µs)

Captured over ~1.5 minutes (153 logged samples / 1,530 ranging cycles),
analyzed with `docs/timing_stats.py`.

## Results

Captured over ~1.5 minutes (153 logged samples, 1 per 10 ranging cycles
= 1,530 total cycles), board sitting idle with all tasks running normally
(ranger, imu_sample, oled_task, button_task).

| Metric | n | mean | sd | min | max |
| --- | --- | --- | --- | --- | --- |
| trig_pulse_us | 153 | 23.0 | 3.31 | 18 | 32 |
| wake_jitter_us | 153 | 11.2 | 2.83 | 7 | 20 |
| period_us | 153 | 60001.6 | 3.60 | 59989 | 60012 |

## Echo pulse-width jitter (derived)

The README's noise measurement (`53cm` target, `n=503`, gathered in step
3) reported a raw distance standard deviation of 2.21mm and a
median-filtered standard deviation of 2.08mm. Converting via
`width_us = mm × 2000 / 343`:

| | sd (mm) | sd (µs, derived) |
| --- | --- | --- |
| raw | 2.21 | 12.9 |
| filtered | 2.08 | 12.1 |

This is noticeably larger than `trig_pulse_us`'s measured sd of 3.31µs
(this document, above) — consistent with the trigger side being pure
software scheduling jitter, while the echo side additionally carries real
acoustic sensor noise. However, this figure is *not* a clean measure of
ISR/capture jitter alone: it also includes any genuine sub-millimeter
movement of the target surface during the original 503-sample capture
window (draft, vibration, minor surface irregularity), and the two
cannot be separated without an independent timing reference on the ECHO
line.

## Interpretation

- `period_us`'s mean and spread show whether the `delay_until` accumulator
  is drift-free over time (see step 1's rationale for using it).
- `wake_jitter_us` reflects real RTIC dispatch latency, including
  contention with `gpio_irq` (equal priority, mutually exclusive on
  Cortex-M — the two cannot literally run at the same instant).
- `trig_pulse_us` quantifies software-side overhead on top of the
  requested 10µs delay; the SRF05 tolerates a longer-than-minimum pulse,
  so this overhead has no functional effect on ranging.

## Known limitation

No oscilloscope or logic analyzer was used. All numbers are software
timestamps from the RP2040's own timer, not independent electrical
measurements — real GPIO rise/fall time and true interrupt-to-pin latency
are not captured here.
