# srf05

[![Crates.io](https://img.shields.io/crates/v/srf05.svg)](https://crates.io/crates/srf05)
[![docs.rs](https://docs.rs/srf05/badge.svg)](https://docs.rs/srf05)

A driver for the **SRF05 ultrasonic rangefinder**, built on `embedded-hal` 1.0.
`no_std`, no allocator, and not tied to any particular MCU, HAL, or framework.

Two ways to get a reading, depending on what your project already has:

- **[`Srf05`]** — a blocking, bit-banged driver. Needs only two GPIO pins and
  a delay implementation. Works on anything `embedded-hal` supports, with no
  interrupts and no timer peripheral required.
- **[`EdgeCapture`]** — a hardware-free primitive for building your own
  interrupt- or event-driven capture. Feed it timestamps from a rising and a
  falling edge, in whatever way your platform delivers them (a GPIO ISR, an
  RTIC task, an async executor's edge-wait future — anything with a
  monotonically increasing microsecond clock), and it hands back a distance.

Both share the same conversion logic ([`pulse_width_to_mm`]), so a reading
means the same thing whichever path produced it.

## Why two APIs

The SRF05 itself doesn't care how you time its echo pulse — but how you *can*
time it varies a lot by project. A quick prototype with no interrupts wired
up wants something that just works when you call it. A project already
built around interrupts (this crate's own motivating project runs a full
RTIC application) wants to feed its own timestamps in and stay in control of
scheduling and priorities. This crate tries to serve both without forcing
either shape on you.

There is currently no async (`embedded-hal-async`) driver. It's planned once
it's been validated against real hardware — see [Roadmap](#roadmap).

## How the sensor works

1. Pulse **TRIG** high for ≥10 µs.
2. The module emits a burst of ultrasonic pulses and raises **ECHO**.
3. ECHO stays high until the echo returns (or ~30 ms elapses with nothing in
   range).
4. Distance is derived from the pulse width: sound travels ~343 m/s, and the
   pulse covers the round trip, so `distance_mm ≈ width_us / 5.83`.

Leave the SRF05's **MODE** pin unconnected — this crate assumes the
separate-trigger-and-echo wiring mode, not the single-pin mode.

## ⚠ ECHO is a 5V signal

The SRF05 requires 5V power, and **ECHO idles and pulses at 5V**, regardless
of your MCU's logic level. On a 3.3V board (Pico, most Cortex-M boards), put
a resistor divider between ECHO and the input pin — a common choice is
1kΩ in series, 2kΩ to ground, giving ≈3.3V at the pin. Skipping this can
damage the GPIO. TRIG can be driven directly from a 3.3V output pin.

## Usage: blocking driver

```rust
use srf05::Srf05;

let mut sensor = Srf05::new(trig_pin, echo_pin, delay);

match sensor.measure() {
    Ok(srf05::Reading::Mm(mm)) => { /* got a distance */ }
    Ok(srf05::Reading::OutOfRange) => { /* valid echo, but nothing within ~4m */ }
    Err(srf05::MeasureError::Reading(srf05::Error::TimeOut)) => {
        /* no echo at all — check wiring/power */
    }
    Err(srf05::MeasureError::Pin(e)) => { /* GPIO operation itself failed */ }
}
```

`trig_pin` and `echo_pin` are any `embedded-hal::digital::{OutputPin, InputPin}`
implementations; `delay` is any `embedded-hal::delay::DelayNs`.

This driver polls the ECHO pin in a loop, using the delay itself as a clock.
It's fully portable, but it busy-waits the CPU for the duration of each
ping — up to ~30ms if nothing is in range. If that cost matters to you (it
usually does on anything running other tasks), use `EdgeCapture` with your
platform's interrupts instead.

## Usage: interrupt-driven capture

```rust
use srf05::EdgeCapture;

let mut capture = EdgeCapture::new();

// In your rising-edge interrupt handler:
capture.on_rise(now_us());

// In your falling-edge interrupt handler:
capture.on_fall(now_us());

// Wherever you collect the result (after triggering a ping and waiting
// at least as long as the sensor's own timeout, ~30-35ms):
match capture.take_reading() {
    Ok(srf05::Reading::Mm(mm)) => { /* ... */ }
    Ok(srf05::Reading::OutOfRange) => { /* ... */ }
    Err(srf05::Error::Timeout) => { /* no complete pulse arrived */ }
}
```

`EdgeCapture` doesn't touch any pins or peripherals itself — you're
responsible for triggering the ping, wiring the two edges to `on_rise`/
`on_fall`, and deciding how long to wait before calling `take_reading`.
In exchange, it fits into whatever scheduling model your project already
uses. It has shipped inside an RTIC application with the echo pin bound to
a shared GPIO interrupt vector alongside other sensors; see the
[originating project](https://github.com/stiiven-dev/srf05-imu-station) for a full
example.

## MSRV

Tested on the current stable Rust toolchain. No specific MSRV is guaranteed yet.

## Testing

```bash
cargo test # host tests: conversion math + embedded-hal-mock–driven Srf05
cargo clippy --all-features -- -D warnings
cargo build --target thumbv6m-none-eabi --no-default-features # confirms no_std
```

The blocking driver's edge-handling logic is exercised against
[`embedded-hal-mock`](https://docs.rs/embedded-hal-mock) pin transactions —
no hardware required to verify it.

## Roadmap

- [ ] Async driver (`embedded-hal-async`'s `Wait` trait), once validated on
      real hardware against at least one HAL with async GPIO support
- [ ] MSRV policy

## License

Dual-licensed under [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE), at your option.
