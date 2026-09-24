#![no_std]
#![no_main]

mod usb_log;
use panic_persist as _; // placeholder; panic-persist comes back once logging is ported

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
const XTAL_FREQ_HZ: u32 = 12_000_000;
///Mailbox filled by the echo interrupt, read by the ranging task.

//-----------------------the RTIC app-----------------------------
#[rtic::app(device = rp2040_hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1])]
mod app {
    use crate::usb_log;
    use crate::XTAL_FREQ_HZ;
    use core::cell::RefCell;
    use embedded_hal::digital::OutputPin;
    use embedded_hal_bus::i2c::RefCellDevice;
    use motion_core::{Calibration, Calibrator, MedianFilter, RawSample3, NO_READING};
    use rp2040_hal::fugit::RateExtU32;
    use rp2040_hal::pac;
    use rp2040_hal::{clocks::init_clocks_and_plls, gpio, Sio, Watchdog};
    use rtic_monotonics::rp2040::prelude::*;
    use srf05::{EdgeCapture, Error as EchoError, Reading};
    use static_cell::StaticCell;

    rp2040_timer_monotonic!(Mono);

    type TrigPin = gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionSioOutput, gpio::PullDown>;
    type EchoPin = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioInput, gpio::PullNone>;
    type StatusLed = gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionSioOutput, gpio::PullDown>;
    type SdaPin = gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionI2C, gpio::PullUp>;
    type SclPin = gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionI2C, gpio::PullUp>;
    type ImuI2c = rp2040_hal::I2C<pac::I2C0, (SdaPin, SclPin)>;
    type ImuIntPin = gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionSioInput, gpio::PullNone>;
    type ButtonPin = gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSioInput, gpio::PullUp>;

    static I2C_BUS: StaticCell<RefCell<rp2040_hal::I2C<pac::I2C0, (SdaPin, SclPin)>>> =
        StaticCell::new();

    #[shared]
    struct Shared {
        echo: EdgeCapture,
        imu_data_ready: bool,
        calibration: Option<Calibration>,
        imu: Mpu6050<RefCellDevice<'static, ImuI2c>>,
    }

    #[local]
    struct Local {
        trig: TrigPin,
        echo_pin: EchoPin,
        led: StatusLed,

        filter: MedianFilter<5>,

        imu_int_pin: ImuIntPin,
        imu: Mpu6050<RefCell<'static, ImuI2c>>,
        calibrator: Calibrator<200>,

        button_pin: ButtonPin,

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

        let i2c: ImuI2c = rp2040_hal::I2C::i2c0(
            cx.device.I2C0,
            sda,
            scl,
            400.kHz(),
            &mut resets,
            &clocks.peripheral_clock,
        );

        let bus: &'static RefCell<_> = I2C_BUS.init(RefCell::new(i2c));
        let imu_i2c = RefCellDevice::new(bus);

        startup::spawn().ok();
        (
            Shared {
                echo: EdgeCapture::new(),
                imu_data_ready: false,
                calibration: None,
                imu,
            },
            Local {
                trig,
                echo_pin,
                led,
                panic_msg,
                filter: MedianFilter::new(),
            },
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 1)]
    fn usb_irq(_cx: usb_irq::Context) {
        usb_log::poll();
    }

    #[task(local = [panic_msg], priority = 1)]
    async fn startup(cx: startup::Context) {
        Mono::delay(3.secs()).await; // let the host attach to the serial port
        defmt::info!("srf05-imu-station is up!");
        if let Some(msg) = cx.local.panic_msg.take() {
            defmt::error!("previous boot panicked: {}", msg);
        }
        ranger::spawn().ok();
    }

    #[task(binds = IO_IRQ_BANK0, priority = 3,local =[echo_pin],shared=[echo] ) ]
    fn gpio_irq(mut cx: gpio_irq::Context) {
        let now = Mono::now().ticks();
        let pin = cx.local.echo_pin;

        if pin.interrupt_status(gpio::Interrupt::EdgeHigh) {
            pin.clear_interrupt(gpio::Interrupt::EdgeHigh);
            cx.shared.echo.lock(|e| e.on_rise(now));
        }

        if pin.interrupt_status(gpio::Interrupt::EdgeLow) {
            pin.clear_interrupt(gpio::Interrupt::EdgeLow);
            cx.shared.echo.lock(|e| e.on_fall(now));
        }
    }

    #[task(local = [trig,led,filter ] , shared = [echo], priority = 2)]
    async fn ranger(mut cx: ranger::Context) {
        let mut next = Mono::now();
        loop {
            cx.shared.echo.lock(|e| e.reset());

            cx.local.trig.set_high().ok();
            Mono::delay(10.micros()).await;
            cx.local.trig.set_low().ok();

            Mono::delay(35.millis()).await;

            // Map to a plain number for the filter: real distances stay as they
            // are, everything that isn't a distance becomes NO_READING.
            let raw = match cx.shared.echo.lock(|e| e.take_reading()) {
                Ok(Reading::Mm(mm)) => mm,
                Ok(Reading::OutOfRange) => NO_READING,
                Err(EchoError::TimeOut) => {
                    defmt::warn!("no echo - check wiring");
                    NO_READING
                }
            };

            // While the window fills (first 4 samples) pass the raw value through
            let filt = cx.local.filter.push(raw).unwrap_or(raw);

            defmt::info!("range raw={=u16} filt={=u16}", raw, filt);

            // The LED now follows the *filtered* value: on when closer than 30cm.
            if filt < 300 {
                cx.local.led.set_high().ok();
            } else {
                cx.local.led.set_low().ok();
            }

            next += 60.millis();
            Mono::delay_until(next).await;
        }
    }
}
