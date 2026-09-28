#![no_std]
#![no_main]

mod debouncer;
mod flash_store;
mod imu;
mod usb_log;

use panic_persist as _;
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H; // was W25Q080 — mismatched your board's actual 2MB chip

const XTAL_FREQ_HZ: u32 = 12_000_000;
const DEBOUNCE_TICKS: u64 = 10_000; // 10 ms
const MULTI_CLICKS_WINDOW_TICKS: u64 = 400_000; // 400 ms
const HOLD_TICKS: u64 = 1_500_000; // 1.5 s
const BUTTON_POLL_TICKS: u64 = 5_000;

#[rtic::app(
    device = rp2040_hal::pac,
    peripherals = true,
    dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1, PIO1_IRQ_0]
)]
mod app {
    use crate::debouncer::{ButtonEvent, ButtonMonitor};
    use crate::{flash_store, imu, usb_log};
    use crate::{
        BUTTON_POLL_TICKS, DEBOUNCE_TICKS, HOLD_TICKS, MULTI_CLICKS_WINDOW_TICKS, XTAL_FREQ_HZ,
    };
    use embedded_graphics::{
        pixelcolor::BinaryColor,
        prelude::*,
        primitives::{Circle, PrimitiveStyle},
    };
    use embedded_hal::digital::OutputPin;
    use motion_core::{
        Attitude, Calibration, Calibrator, ComplementaryFilter, MedianFilter, NO_READING,
    };
    use rp2040_hal::fugit::RateExtU32;
    use rp2040_hal::{clocks::init_clocks_and_plls, gpio, Sio, Watchdog};
    use rp2040_hal::{pac, rom_data};
    use rtic_monotonics::rp2040::prelude::*;
    use srf05::{EdgeCapture, Error as EchoError, Reading};
    use ssd1306::{mode::DisplayConfig, prelude::*, I2CDisplayInterface, Ssd1306};

    const MAX_ANGLE_DEG: f32 = 30.0; //tilt at which the bubble reaches the limit
    const CENTER: Point = Point::new(64, 32);
    const BOUNDARY_RADIUS: i32 = 28;
    const BUBBLE_RADIUS: u32 = 6;

    const WATCHDOG_FEED_TICKS: u64 = 200_000; // 200ms — comfortably under the 500ms deadline

    rp2040_timer_monotonic!(Mono);

    type TrigPin = gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionSioOutput, gpio::PullDown>;
    type EchoPin = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioInput, gpio::PullNone>;
    type StatusLed = gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionSioOutput, gpio::PullDown>;
    type SdaPin = gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionI2C, gpio::PullUp>;
    type SclPin = gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionI2C, gpio::PullUp>;
    // Named for the peripheral, not the IMU — the OLED will share this same bus.
    type SharedI2c = rp2040_hal::I2C<pac::I2C0, (SdaPin, SclPin)>;
    type ImuIntPin = gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionSioInput, gpio::PullDown>;
    type ButtonPin = gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSioInput, gpio::PullUp>;

    #[shared]
    struct Shared {
        echo: EdgeCapture,
        calibrating: bool,
        calibration: Option<Calibration>,
        // The bus itself, owned outright — not wrapped in RefCell, not behind
        // a 'static reference. RTIC's own lock() serializes access; whichever
        // task needs the bus (IMU today, OLED later) locks it, does its
        // transaction, and releases it. No driver owns the bus exclusively.
        i2c_bus: SharedI2c,
        attitude: Attitude,
        imu_data_ready: bool,
        imu_heartbeat: u32,
        oled_heartbeat: u32,
        button_heartbeat: u32,
    }

    #[local]
    struct Local {
        trig: TrigPin,
        echo_pin: EchoPin,
        led: StatusLed,
        median_filter: MedianFilter<5>,
        imu_int_pin: ImuIntPin,
        calibrator: Calibrator<200>,
        comp_filter: ComplementaryFilter,
        button: ButtonMonitor<ButtonPin>,
        panic_msg: Option<&'static str>,
        watchdog: Watchdog,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let panic_msg = panic_persist::get_panic_message_utf8();

        let mut resets = cx.device.RESETS;
        let mut watchdog = Watchdog::new(cx.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XTAL_FREQ_HZ,
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        Mono::start(cx.device.TIMER, &resets);

        usb_log::init(
            cx.device.USBCTRL_REGS,
            cx.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            &mut resets,
        );

        let sio = Sio::new(cx.device.SIO);
        let pins = gpio::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let led = pins.gpio13.into_push_pull_output();
        let trig = pins
            .gpio15
            .into_push_pull_output_in_state(gpio::PinState::Low);
        let echo_pin = pins.gpio14.into_floating_input();
        echo_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);
        echo_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

        let sda: SdaPin = pins.gpio4.reconfigure();
        let scl: SclPin = pins.gpio5.reconfigure();
        let i2c_bus: SharedI2c = rp2040_hal::I2C::i2c0(
            cx.device.I2C0,
            sda,
            scl,
            400.kHz(),
            &mut resets,
            &clocks.peripheral_clock,
        );

        let imu_int_pin: ImuIntPin = pins.gpio16.into_pull_down_input();
        imu_int_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

        let button_pin: ButtonPin = pins.gpio12.into_pull_up_input();
        let now0 = Mono::now().ticks();
        let button = ButtonMonitor::new(
            button_pin,
            true,
            DEBOUNCE_TICKS,
            MULTI_CLICKS_WINDOW_TICKS,
            HOLD_TICKS,
            now0,
        )
        .expect("button pin read is infallible on this HAL");

        let comp_filter = ComplementaryFilter::new(0.98);

        watchdog.start(rp2040_hal::fugit::MicrosDurationU32::millis(1000));

        watchdog_task::spawn().ok();
        heartbeat_log_task::spawn().ok();
        button_task::spawn().ok();
        startup::spawn().ok();
        (
            Shared {
                echo: EdgeCapture::new(),
                calibrating: false,
                calibration: None,
                i2c_bus,
                attitude: Attitude::default(),
                imu_data_ready: false,
                imu_heartbeat: 0,
                oled_heartbeat: 0,
                button_heartbeat: 0,
            },
            Local {
                trig,
                echo_pin,
                led,
                median_filter: MedianFilter::new(),
                imu_int_pin,
                calibrator: Calibrator::new(),
                comp_filter,
                button,
                panic_msg,
                watchdog,
            },
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 3)]
    fn usb_irq(_cx: usb_irq::Context) {
        usb_log::poll();
    }

    #[task(local = [panic_msg], shared = [i2c_bus,calibration], priority = 1)]
    async fn startup(mut cx: startup::Context) {
        Mono::delay(3.secs()).await;
        if let Some(msg) = cx.local.panic_msg.take() {
            defmt::error!("previous boot panicked: {}", msg);
        }
        let init_ok = cx.shared.i2c_bus.lock(|i2c| imu::init(i2c));
        match init_ok {
            Ok(()) => {
                cx.shared.i2c_bus.lock(|i2c| {
                    let interface = I2CDisplayInterface::new(i2c);
                    let mut display =
                        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
                            .into_buffered_graphics_mode();
                    if display.init().is_err() {
                        defmt::warn!("oled init failed");
                    }
                });

                Mono::delay(200.millis()).await;

                ranger::spawn().ok();
                imu_sample::spawn().ok();
                oled_task::spawn().ok();
                filter_benchmark::spawn().ok();
            }
            Err(e) => defmt::error!("IMU init failed: {:?}", defmt::Debug2Format(&e)),
        }
        if let Some(cal) = flash_store::load_calibration().await {
            defmt::info!("loaded stored calibration");
            cx.shared.calibration.lock(|c| *c = Some(cal));
        } else {
            defmt::info!("no stored calibration - hold the button to calibrate");
        }
    }

    #[task(binds = IO_IRQ_BANK0, priority = 3,
           local = [echo_pin, imu_int_pin], shared = [echo, imu_data_ready])]
    fn gpio_irq(mut cx: gpio_irq::Context) {
        let now = Mono::now().ticks();

        let echo = cx.local.echo_pin;
        if echo.interrupt_status(gpio::Interrupt::EdgeHigh) {
            echo.clear_interrupt(gpio::Interrupt::EdgeHigh);
            cx.shared.echo.lock(|e| e.on_rise(now));
        }
        if echo.interrupt_status(gpio::Interrupt::EdgeLow) {
            echo.clear_interrupt(gpio::Interrupt::EdgeLow);
            cx.shared.echo.lock(|e| e.on_fall(now));
        }

        let imu_int = cx.local.imu_int_pin;
        if imu_int.interrupt_status(gpio::Interrupt::EdgeHigh) {
            imu_int.clear_interrupt(gpio::Interrupt::EdgeHigh);
            cx.shared.imu_data_ready.lock(|f| *f = true)
        }
    }

    #[task(local=[log_counter: u32 =0,comp_filter , last_sample: Option<fugit::TimerInstantU64<1_000_000>> = None],shared = [ i2c_bus,calibration,calibrating,attitude,imu_data_ready,imu_heartbeat], priority = 1)]
    async fn imu_sample(mut cx: imu_sample::Context) {
        loop {
            loop {
                let ready = cx
                    .shared
                    .imu_data_ready
                    .lock(|ready| core::mem::replace(ready, false));
                if ready {
                    break;
                }
                Mono::delay(1.millis()).await;
            }
            let now = Mono::now();
            let dt_s = match cx.local.last_sample.replace(now) {
                Some(prev) => (now - prev).to_micros() as f32 / 1_000_000.0,
                None => 0.01, //first sample assume the nominal 100Hz period
            };

            let reading = cx.shared.i2c_bus.lock(|i2c| imu::read(i2c));

            let (accel_raw, gyro_raw) = match reading {
                Ok(v) => v,
                Err(e) => {
                    defmt::warn!("imu read failed: {:?}", defmt::Debug2Format(&e));
                    continue; // back to outer loop, wait for next data-ready flag
                }
            };

            let (accel, gyro) = cx.shared.calibration.lock(|cal| match cal {
                Some(c) => (c.correct_accel(accel_raw), c.correct_gyro(gyro_raw)),
                None => (accel_raw, gyro_raw),
            });

            *cx.local.log_counter += 1;
            if *cx.local.log_counter % 50 == 0 && !cx.shared.calibrating.lock(|c| *c) {
                defmt::info!(
                    "accel=({=i16},{=i16},{=i16}) gyro=({=i16},{=i16},{=i16})",
                    accel.x,
                    accel.y,
                    accel.z,
                    gyro.x,
                    gyro.y,
                    gyro.z
                );
            }
            let attitude =
                cx.local
                    .comp_filter
                    .update(dt_s, gyro.gyro_to_dps(), accel.accel_to_g());
            cx.shared.attitude.lock(|a| *a = attitude);
            cx.shared.imu_heartbeat.lock(|h| *h = h.wrapping_add(1));
        }
    }

    #[task(local = [calibrator], shared = [calibrating,calibration, i2c_bus], priority = 2)]
    async fn calibrate(mut cx: calibrate::Context) {
        defmt::info!("calibrating: settling...");
        cx.shared.calibrating.lock(|c| *c = true);
        Mono::delay(500.millis()).await;

        defmt::info!("calibrating: hold the board still...");
        loop {
            let reading = cx.shared.i2c_bus.lock(|i2c| imu::read(i2c));
            match reading {
                Ok((accel, gyro)) => {
                    if let Some(result) = cx.local.calibrator.push(gyro, accel) {
                        defmt::info!(
                            "calibration done: gyro_bias=({=i16},{=i16},{=i16}) accel_offset=({=i16},{=i16},{=i16})",
                            result.gyro_bias.x, result.gyro_bias.y, result.gyro_bias.z,
                            result.accel_offset.x, result.accel_offset.y, result.accel_offset.z
                        );
                        cx.shared.calibration.lock(|c| *c = Some(result));
                        flash_store::store_calibration(&result).await;
                        cx.shared.calibrating.lock(|c| *c = false);
                        return;
                    }
                }

                Err(_) => {
                    defmt::warn!("calibration read failed");
                    cx.shared.calibrating.lock(|c| *c = false);
                }
            }
            Mono::delay(10.millis()).await;
        }
    }

    #[task(local = [trig, led, median_filter,log_counter: u32 =0,last_cycle_start: Option<fugit::TimerInstantU64<1_000_000>> = None], shared = [echo], priority = 3)]
    async fn ranger(mut cx: ranger::Context) {
        let mut next = Mono::now();
        loop {
            let wake_actual = Mono::now();
            let wake_jitter_us = (wake_actual - next).to_micros();

            let period_us = match cx.local.last_cycle_start.replace(wake_actual) {
                Some(prev) => (wake_actual - prev).to_micros(),
                None => 60_000, // first cycle: no prior cycle to compare against
            };
            cx.shared.echo.lock(|e| e.reset());

            let t_trig_start = Mono::now();
            cx.local.trig.set_high().ok();
            Mono::delay(10.micros()).await;
            cx.local.trig.set_low().ok();
            let trig_pulse_us = (Mono::now() - t_trig_start).to_micros();

            Mono::delay(35.millis()).await;

            let raw = match cx.shared.echo.lock(|e| e.take_reading()) {
                Ok(Reading::Mm(mm)) => mm,
                Ok(Reading::OutOfRange) => NO_READING,
                Err(EchoError::TimeOut) => {
                    defmt::warn!("no echo - check wiring");
                    NO_READING
                }
            };

            let filt = cx.local.median_filter.push(raw).unwrap_or(raw);

            *cx.local.log_counter += 1;
            if *cx.local.log_counter % 10 == 0 {
                defmt::info!(
                "range raw={=u16} filt={=u16} timing trig_us={=u64} wake_jitter_us={=u64} period_us={=u64}",
                raw, filt, trig_pulse_us, wake_jitter_us, period_us
            );
            }

            if filt < 300 {
                cx.local.led.set_high().ok();
            } else {
                cx.local.led.set_low().ok();
            }

            next += 60.millis();
            Mono::delay_until(next).await;
        }
    }

    #[task(shared= [button_heartbeat],local = [button], priority = 1)]
    async fn button_task(mut cx: button_task::Context) {
        loop {
            let now = Mono::now().ticks();
            match cx.local.button.update(now) {
                Ok(ButtonEvent::HoldTriggered) => {
                    defmt::info!("button held - starting calibration");
                    calibrate::spawn().ok();
                }
                Ok(ButtonEvent::Clicks(3)) => {
                    defmt::warn!("resetting into BOOTSEL !");
                    Mono::delay(50.millis()).await;
                    rom_data::reset_to_usb_boot(0, 0);
                }
                Ok(ButtonEvent::Clicks(n)) => {
                    defmt::info!("button clicked {=u8} times", n);
                }
                Ok(ButtonEvent::None) => {}
                Err(_) => {} // Infallible on this HAL's InputPin
            }
            cx.shared.button_heartbeat.lock(|h| *h = h.wrapping_add(1));
            Mono::delay(BUTTON_POLL_TICKS.micros()).await;
        }
    }

    #[task(shared = [attitude,i2c_bus,oled_heartbeat],priority = 1)]
    async fn oled_task(mut cx: oled_task::Context) {
        let mut next = Mono::now();
        loop {
            let attitude = cx.shared.attitude.lock(|a| *a);

            let travel = (BOUNDARY_RADIUS - BUBBLE_RADIUS as i32) as f32;
            let dx = -(attitude.roll_deg / MAX_ANGLE_DEG).clamp(-1.0, 1.0) * travel;
            let dy = (attitude.pitch_deg / MAX_ANGLE_DEG).clamp(-1.0, 1.0) * travel;
            let bubble_center = CENTER + Point::new(dx as i32, dy as i32);

            cx.shared.i2c_bus.lock(|i2c| {
                let interface = I2CDisplayInterface::new(i2c);
                let mut display =
                    Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
                        .into_buffered_graphics_mode();
                display.clear(BinaryColor::Off).ok();

                Circle::with_center(CENTER, (BOUNDARY_RADIUS * 2) as u32)
                    .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
                    .draw(&mut display)
                    .ok();
                Circle::with_center(bubble_center, BUBBLE_RADIUS * 2)
                    .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                    .draw(&mut display)
                    .ok();

                display.flush().ok();
            });
            cx.shared.oled_heartbeat.lock(|h| *h = h.wrapping_add(1));
            next += 100.millis();
            Mono::delay_until(next).await;
        }
    }

    #[task(local = [watchdog], priority = 3)]
    async fn watchdog_task(cx: watchdog_task::Context) {
        loop {
            cx.local.watchdog.feed();
            Mono::delay(WATCHDOG_FEED_TICKS.micros()).await;
        }
    }

    #[task(local = [last_imu: u32 = 0, last_oled: u32 = 0, last_button: u32 = 0],
       shared = [imu_heartbeat, oled_heartbeat, button_heartbeat], priority = 1)]
    async fn heartbeat_log_task(mut cx: heartbeat_log_task::Context) {
        loop {
            let imu = cx.shared.imu_heartbeat.lock(|h| *h);
            let oled = cx.shared.oled_heartbeat.lock(|h| *h);
            let button = cx.shared.button_heartbeat.lock(|h| *h);

            defmt::info!(
                "alive: imu+{=u32} oled+{=u32} button+{=u32}",
                imu.wrapping_sub(*cx.local.last_imu),
                oled.wrapping_sub(*cx.local.last_oled),
                button.wrapping_sub(*cx.local.last_button),
            );
            *cx.local.last_imu = imu;
            *cx.local.last_oled = oled;
            *cx.local.last_button = button;

            Mono::delay(1.secs()).await;
        }
    }

    #[task(priority = 1)]
    async fn filter_benchmark(_cx: filter_benchmark::Context) {
        const ITERS: u32 = 2000;
        let gyro = (12i16, -8, 3);
        let accel = (150i16, -80, 16300);

        let mut f32_filter = motion_core::ComplementaryFilter::new(0.98);
        let t0 = Mono::now();
        for _ in 0..ITERS {
            let g = (
                gyro.0 as f32 / 131.0,
                gyro.1 as f32 / 131.0,
                gyro.2 as f32 / 131.0,
            );
            let a = (
                accel.0 as f32 / 16384.0,
                accel.1 as f32 / 16384.0,
                accel.2 as f32 / 16384.0,
            );
            core::hint::black_box(f32_filter.update(0.01, g, a));
        }
        let f32_total_us = (Mono::now() - t0).to_micros();

        let mut fixed_filter = motion_core::FixedComplementaryFilter::new(0.98);
        let t1 = Mono::now();
        for _ in 0..ITERS {
            core::hint::black_box(fixed_filter.update(10_000, gyro, accel));
        }
        let fixed_total_us = (Mono::now() - t1).to_micros();

        defmt::info!(
            "benchmark: f32 total_us={=u64} avg_ns={=u32}  fixed total_us={=u64} avg_ns={=u32}",
            f32_total_us,
            (f32_total_us * 1000 / ITERS as u64) as u32,
            fixed_total_us,
            (fixed_total_us * 1000 / ITERS as u64) as u32,
        );
    }
}
