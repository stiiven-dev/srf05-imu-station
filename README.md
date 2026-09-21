# srf05-imu-station

Ultrasonic rangefinder + IMU attitude estimation on one Pico W, running on RTIC v2 — no debug probe required. Includes a published, reusable `srf05` driver crate.

<!-- TODO: hero photo of the breadboard -->
`docs/images/breadboard.jpg`

<!-- TODO: demo GIF — distance readout, page-flip to the bubble level, tilting the board -->
`docs/images/demo.gif`

---

## Features

- **Interrupt-driven SRF05 ranging** — timer task pings every 60ms, GPIO ISR captures both
echo edges, no busy-waiting on the CPU. Median-of-5 filtered, with a real timeout so
an out-of-range reading never hangs the firmware.
- **MPU-6050 attitude estimation** — startup calibration (gyro bias + Accel offsets), complementary
filter for pitch/roll, animated bubble level on the OLED. Calibration persists to flash
via `sequential-storage`, survives power-cycle.
- **INT-pin-driven IMU sampling**, not polling.
- **`crates/srf05` is a real, published, reusable driver** — generic
over `embedded_hal::digital` + `delay`/timer traits, blocking and async variants, `no_std`,
`embedded-hal-mock` tested. Not tied to this project's wiring or RTIC setup.
- Fixed-point vs `f32` benchmark table (see `docs/timing.md`) — the IMU filter runs in
an ISR-adjacent context, where the no-FPU cost actually matters.
- Same USB-only dev loop as every project before this one: `defmt-serial` logging,
`panic-persist` crash capture, no SWD probe.

## To-Do list

- [x] RTIC v2 skeleton — built once, shared by both sensors
- [x] SRF05 interrupt-driven edge capture, timeout + out-of-range handling
- [ ] median-of-5 distance filter, host-tested, measured noise reduction documented
- [ ] extract `srf05` to its own crate, `embedded-hal-mock` tests, `cargo publish --dry-run`
- [ ] IMU calibration routine + flash persistence
- [ ] complementary filter + OLED bubble level, INT-driven sampling
- [ ] `docs/timing.md` with trigger/echo jitter numbers
- [ ] fixed-point vs f32 benchmark table
- [ ] publish `srf05` to crates.io + tag `v1.0.0`

## Hardware

| Part                                  | Qty    | Notes                                                                |
|---------------------------------------|--------|----------------------------------------------------------------------|
| Raspberry Pi Pico W (or WH)           | 1      | RP2040 + CYW43439                                                    |
| SRF05 ultrasonic rangefinder          | 1      | **5V only** — the only 5V part in the whole series                   |
| MPU-6050 (GY-521) breakout            | 1      | 3.3V — power from the Pico's 3V3 pin, **not VBUS**                   |
| SSD1306/SSD1309 0.96" 128×64 I²C OLED | 1      | reused                                                               |
| Push button                           | 1      | calibration hold, same UX as `pico-pot-meter`/`pico-weather-station` |
| Resistors: 1kΩ, 2kΩ                   | 1 each | ECHO level-shift divider                                             |
| Breadboard + jumper wires             | —      |                                                                      |
| USB micro-B cable                     | 1      | data-capable                                                         |

No debug probe needed — same USB-only workflow as every prior project.

## Wiring

| Pico W pin    | Signal                          | Notes                                                                                                                                                            |
|---------------|---------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| GP15 (pin 20) | SRF05 TRIG                      | 3.3V logic, no divider needed                                                                                                                                    |
| GP14 (pin 19) | SRF05 ECHO, via 1kΩ/2kΩ divider | **ECHO is 5V — the divider is not optional, it will damage the GPIO without it**                                                                                 |
| GP4 (pin 6)   | I²C0 SDA                        | shared: MPU-6050 + OLED                                                                                                                                          |
| GP5 (pin 7)   | I²C0 SCL                        | shared: MPU-6050 + OLED                                                                                                                                          |
| GP16 (pin 21) | MPU-6050 INT                    | interrupt-driven sampling, not polled                                                                                                                            |
| GP12 (pin 16) | Button, other leg to GND        | internal pull-up                                                                                                                                                 |
| 3V3 (pin 36)  | MPU-6050 VCC + OLED VCC         | **MPU-6050 from 3V3, not VBUS** — many clone modules back-drive 3.3V GPIOs otherwise                                                                             |
| VBUS (pin 40) | SRF05 VCC                       | **5V, not 3V3** — SRF05 won't run reliably at 3.3V                                                                                                               |
| GND (pin 38)  | Common ground                   | every module, no exceptions — this is the one project in the series mixing 5V and 3.3V sections, so missing ground here is the likeliest cause of anything weird |

<!-- TODO: docs/wiring.md with the full diagram -->

## Quickstart

```bash
cargo run --release   # from firmware/, per the workspace .cargo/config.toml
./watch-defmt.sh
```

Hold the button while moving the IMU through its extremes to
calibrate (same pattern as the pot-meter's calibration UX), release to store. Turn or tilt
the board to see the bubble level track it; watch the distance readout on the SRF05 page respond
as you move something in front of it.

## Architecture

```text
srf05-imu-station/
├── crates/
│   ├── srf05/          # published driver crate — generic over embedded-hal,
│   │                    #   no_std, blocking + async, embedded-hal-mock tested.
│   │                    #   Not specific to this project's wiring.
│   └── motion-core/     # host-testable pure logic — complementary filter,
│                         #   calibration math, median-of-5 filter, verified
│                         #   against known reference angles (see Testing).
│                         #   Same hardware-free pattern as pot-core/station-core.
└── firmware/
    └── src/
        ├── main.rs       # RTIC v2 app: task definitions, resource wiring
        └── ui/           # Page state machine — reused pattern from
                            #   pico-weather-station (page 1 = range, page 2 = level)
```

Two different kinds of crate here, worth being clear about the distinction: `srf05` is a *driver* — it genuinely does I/O, just abstracted
over `embedded-hal` traits so it's reusable and mockable rather than hardwired to this project's
peripherals. `motion-core` is *pure logic* — no I/O at all, same category
as `pot-core` and `station-core` from the earlier projects. Both are testable on the host, but for different
reasons: the driver via mocked hardware, the core crate because it never touches hardware in the
first place.

## Testing

```bash
cargo test -p srf05 --target x86_64-unknown-linux-gnu         # embedded-hal-mock tests, no real hardware
cargo test -p motion-core --target x86_64-unknown-linux-gnu   # pure logic: median filter, complementary filter,
                              #   calibration math — reference-value assertions,
                              #   same style as pot-core/station-core
cargo clippy --workspace --all-features -- -D warnings
cargo fmt --all -- --check
```

No replay-from-hardware test harness in this project — that specific
skill (capturing real sensor data and validating a filter against it offline) was already
demonstrated end-to-end in `pico-pot-meter`'s filter-comparison work, so reproving it here would
cost real time for no new portfolio evidence. `motion-core`'s tests instead check the
complementary filter against known reference angles (e.g. board held level should
read ~0°/0°, tilted 90° should read accordingly) — synthetic but still meaningful, same category
of test as `pot-core`/`station-core`'s reference-value assertions.

`firmware/` has no host tests, same reasoning as every prior project — RTIC task scheduling and real interrupt timing need actual hardware to verify.

## Known limitations

- No proximity buzzer — cut for lack of hardware on hand, not
a design choice. Revisit if a piezo buzzer gets added to the kit later; the wiring/PWM
pattern from the roadmap is still there if needed.
- No IMU replay-test harness — deliberately not reproving
a skill (offline filter validation against captured hardware data) already
shipped in `pico-pot-meter`. `motion-core`'s reference-angle tests cover correctness instead.
- No debug probe support — see `blinky-plus` for what that means in practice for this series.
- The SRF05/MPU-6050 combination shares one repo but the two sensors don't currently
interact (no fused range+attitude use case) — they're combined for build-time
efficiency (one RTIC migration instead of two), not because the sensing tasks are related.

## License

Firmware: dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.
`crates/srf05` carries the same license independently, since it's published standalone.
