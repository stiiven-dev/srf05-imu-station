# Changelog

## 1.0.0

Initial stable release.

- `Srf05`: blocking, bit-banged driver. `embedded-hal` 1.0, no interrupts required.
- `EdgeCapture`: hardware-free mailbox for interrupt/event-driven capture.
- `pulse_width_to_mm` / `Reading` / `Error`.
- Tested against `embedded-hal-mock`; no hardware required to run the test suite.

### Planned (see README roadmap)

- Async driver (`embedded-hal-async`'s `Wait` trait), pending validation on
  a HAL with async GPIO support.
  