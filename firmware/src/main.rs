#![no_std]
#![no_main]

use panic_halt as _; // placeholder; panic-persist comes back once logging is ported

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rtic::app(device = rp2040_hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1])]
mod app {
    use embedded_hal::digital::StatefulOutputPin;
    use rp2040_hal::{clocks::init_clocks_and_plls, gpio, Sio, Watchdog};
    use rtic_monotonics::rp2040::prelude::*;
    const XTAL_FREQ_HZ: u32 = 12_000_000;

    rp2040_timer_monotonic!(Mono);

    type Heartbeat = gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionSioOutput, gpio::PullDown>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: Heartbeat,
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

        heartbeat::spawn().ok();
        (Shared {}, Local { led })
    }

    #[task(local = [led], priority=1 ) ]
    async fn heartbeat(cx: heartbeat::Context) {
        loop {
            cx.local.led.toggle().ok();
            Mono::delay(500.millis()).await;
        }
    }
}
