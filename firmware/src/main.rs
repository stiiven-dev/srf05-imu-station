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
    use crate::usb_log;
    use crate::{flash_store, imu};
    use crate::{
        BUTTON_POLL_TICKS, DEBOUNCE_TICKS, HOLD_TICKS, MULTI_CLICKS_WINDOW_TICKS, XTAL_FREQ_HZ,
    };
    use embedded_hal::digital::OutputPin;
    use motion_core::{Calibration, Calibrator, MedianFilter, NO_READING};
    use rp2040_hal::fugit::RateExtU32;
    use rp2040_hal::{clocks::init_clocks_and_plls, gpio, Sio, Watchdog};
    use rp2040_hal::{pac, rom_data};
    use rtic_monotonics::rp2040::prelude::*;
    use srf05::{EdgeCapture, Error as EchoError, Reading};

    rp2040_timer_monotonic!(Mono);

    type TrigPin = gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionSioOutput, gpio::PullDown>;
    type EchoPin = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioInput, gpio::PullNone>;
    type StatusLed = gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionSioOutput, gpio::PullDown>;
    type SdaPin = gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionI2C, gpio::PullUp>;
    type SclPin = gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionI2C, gpio::PullUp>;
    // Named for the peripheral, not the IMU — the OLED will share this same bus.
    type SharedI2c = rp2040_hal::I2C<pac::I2C0, (SdaPin, SclPin)>;
    type ImuIntPin = gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionSioInput, gpio::PullNone>;
    type ButtonPin = gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSioInput, gpio::PullUp>;

    #[shared]
    struct Shared {
        echo: EdgeCapture,
        imu_data_ready: bool,
        calibrating: bool,
        calibration: Option<Calibration>,
        // The bus itself, owned outright — not wrapped in RefCell, not behind
        // a 'static reference. RTIC's own lock() serializes access; whichever
        // task needs the bus (IMU today, OLED later) locks it, does its
        // transaction, and releases it. No driver owns the bus exclusively.
        i2c_bus: SharedI2c,
    }

    #[local]
    struct Local {
        trig: TrigPin,
        echo_pin: EchoPin,
        led: StatusLed,
        filter: MedianFilter<5>,
        imu_int_pin: ImuIntPin,
        calibrator: Calibrator<200>,
        button: ButtonMonitor<ButtonPin>,
        panic_msg: Option<&'static str>,
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
        let mut i2c_bus: SharedI2c = rp2040_hal::I2C::i2c0(
            cx.device.I2C0,
            sda,
            scl,
            100.kHz(),
            &mut resets,
            &clocks.peripheral_clock,
        );

        let imu_int_pin: ImuIntPin = pins.gpio16.into_floating_input();
        imu_int_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

        // init() runs here, before anything is shared or locked — init owns
        // everything outright until the moment it returns.
        imu::init(&mut i2c_bus).expect("MPU-6050 init failed, check wiring/power");

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

        button_task::spawn().ok();
        startup::spawn().ok();
        (
            Shared {
                echo: EdgeCapture::new(),
                imu_data_ready: false,
                calibrating: false,
                calibration: None,
                i2c_bus,
            },
            Local {
                trig,
                echo_pin,
                led,
                filter: MedianFilter::new(),
                imu_int_pin,
                calibrator: Calibrator::new(),
                button,
                panic_msg,
            },
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 3)]
    fn usb_irq(_cx: usb_irq::Context) {
        usb_log::poll();
    }

    #[task(local = [panic_msg], shared = [calibration], priority = 1)]
    async fn startup(mut cx: startup::Context) {
        Mono::delay(3.secs()).await;
        defmt::info!("srf05-imu-station is up!");
        if let Some(msg) = cx.local.panic_msg.take() {
            defmt::error!("previous boot panicked: {}", msg);
        }

        if let Some(cal) = flash_store::load_calibration().await {
            defmt::info!("loaded stored calibration");
            cx.shared.calibration.lock(|c| *c = Some(cal));
        } else {
            defmt::info!("no stored calibration - hold the button to calibrate");
        }

        ranger::spawn().ok();
        imu_sample::spawn().ok();
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
            cx.shared.imu_data_ready.lock(|ready| *ready = true);
        }
    }

    #[task(local=[log_counter: u32 =0],shared = [imu_data_ready, i2c_bus,calibration,calibrating], priority = 1)]
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

            let reading = cx.shared.i2c_bus.lock(|i2c| imu::read(i2c));
            match reading {
                Ok((accel_raw, gyro_raw)) => {
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
                }
                Err(error) => defmt::warn!("imu read failed: {:?}", defmt::Debug2Format(&error)),
            }
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

    #[task(local = [trig, led, filter,log_counter: u32 =0], shared = [echo], priority = 3)]
    async fn ranger(mut cx: ranger::Context) {
        let mut next = Mono::now();
        loop {
            cx.shared.echo.lock(|e| e.reset());

            cx.local.trig.set_high().ok();
            Mono::delay(10.micros()).await;
            cx.local.trig.set_low().ok();

            Mono::delay(35.millis()).await;

            let raw = match cx.shared.echo.lock(|e| e.take_reading()) {
                Ok(Reading::Mm(mm)) => mm,
                Ok(Reading::OutOfRange) => NO_READING,
                Err(EchoError::TimeOut) => {
                    defmt::warn!("no echo - check wiring");
                    NO_READING
                }
            };

            let filt = cx.local.filter.push(raw).unwrap_or(raw);

            *cx.local.log_counter += 1;
            if *cx.local.log_counter % 10 == 0 {
                defmt::info!("range raw={=u16} filt={=u16}", raw, filt);
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

    #[task(local = [button], priority = 1)]
    async fn button_task(cx: button_task::Context) {
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
            Mono::delay(BUTTON_POLL_TICKS.micros()).await;
        }
    }
}
