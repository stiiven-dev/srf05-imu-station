#![no_std]
#![no_main]

use panic_halt as _; // placeholder; panic-persist comes back once logging is ported

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
const XTAL_FREQ_HZ: u32 = 12_000_000;
///Mailbox filled by the echo interrupt, read by the ranging task.
#[derive(Clone, Copy)]
pub struct EchoCapture {
    rise_us: Option<u64>,
    width_us: Option<u32>,
}

impl EchoCapture {
    pub const fn new() -> Self {
        Self {
            rise_us: None,
            width_us: None,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn on_rise(&mut self, now_us: u64) {
        self.rise_us = Some(now_us);
    }

    pub fn on_fall(&mut self, now_us: u64) {
        // A falling edge with no prior rising edge is noise , ignore it.
        if let Some(rise) = self.rise_us.take() {
            let w = now_us.saturating_sub(rise);
            self.width_us = Some(u32::try_from(w).unwrap_or(u32::MAX));
        }
    }

    pub fn take_result(&mut self) -> Option<u32> {
        self.width_us.take()
    }
}

#[derive(Clone, Copy)]
pub enum Reading {
    Mm(u16),
    OutOfRange,
    NoEcho,
}

const MAX_RANGE_MM: u64 = 4000;

pub fn classify(width_us: Option<u32>) -> Reading {
    match width_us {
        None => Reading::NoEcho,
        Some(w) => {
            let mm = (w.min(100_000) as u64) * 343 / 2000;
            if mm > MAX_RANGE_MM {
                Reading::OutOfRange
            } else {
                Reading::Mm(mm as u16)
            }
        }
    }
}

//-----------------------the RTIC app-----------------------------
#[rtic::app(device = rp2040_hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1])]
mod app {

    use super::{classify, EchoCapture, Reading, XTAL_FREQ_HZ};
    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use rp2040_hal::{clocks::init_clocks_and_plls, gpio, Sio, Watchdog};
    use rtic_monotonics::rp2040::prelude::*;

    rp2040_timer_monotonic!(Mono);

    type TrigPin = gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionSioOutput, gpio::PullDown>;
    type EchoPin = gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionSioInput, gpio::PullNone>;
    type StatusLed = gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionSioOutput, gpio::PullDown>;

    #[shared]
    struct Shared {
        echo: EchoCapture,
    }

    #[local]
    struct Local {
        trig: TrigPin,
        echo_pin: EchoPin,
        led: StatusLed,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut resets = cx.device.RESETS;
        let mut watchdog = Watchdog::new(cx.device.WATCHDOG);
        let _clocks = init_clocks_and_plls(
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

        ranger::spawn().ok();
        (
            Shared {
                echo: EchoCapture::new(),
            },
            Local {
                trig,
                echo_pin,
                led,
            },
        )
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

    #[task(local = [trig,led] , shared = [echo], priority = 2)]
    async fn ranger(mut cx: ranger::Context) {
        let mut next = Mono::now();
        loop {
            cx.shared.echo.lock(|e| e.reset());

            cx.local.trig.set_high().ok();
            Mono::delay(10.micros()).await;
            cx.local.trig.set_low().ok();

            Mono::delay(35.millis()).await;

            let width = cx.shared.echo.lock(|e| e.take_result());
            match classify(width) {
                Reading::Mm(mm) if mm < 3000 => {
                    cx.local.led.set_high().ok();
                }
                Reading::NoEcho => {
                    cx.local.led.toggle().ok();
                }
                _ => {
                    cx.local.led.set_low().ok();
                }
            }

            next += 60.millis();
            Mono::delay_until(next).await;
        }
    }
}
