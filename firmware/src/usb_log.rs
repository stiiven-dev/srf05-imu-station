//! USB CDC serial transport for defmt (no debug probe needed).
//!
//! The USB device lives in a global because `defmt` macros can be called from
//! anywhere and have no access to RTIC resources.

use core::cell::RefCell;
use critical_section::Mutex;
use rp2040_hal::{clocks::UsbClock, pac, usb::UsbBus};
use static_cell::StaticCell;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

type UsbState = (UsbDevice<'static, UsbBus>, SerialPort<'static, UsbBus>);

static USB_STATE: Mutex<RefCell<Option<UsbState>>> = Mutex::new(RefCell::new(None));
static USB_BUS: StaticCell<UsbBusAllocator<UsbBus>> = StaticCell::new();
static WRITER: StaticCell<DefmtUsbWriter> = StaticCell::new();

struct DefmtUsbWriter;

impl embedded_io::ErrorType for DefmtUsbWriter {
    type Error = core::convert::Infallible;
}

impl embedded_io::Write for DefmtUsbWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut written = 0;
        let mut idle_polls = 0u32;
        const MAX_IDLE_POLLS: u32 = 2_000;

        while written < buf.len() && idle_polls < MAX_IDLE_POLLS {
            let mut give_up = false;

            critical_section::with(|cs| {
                if let Some((usb_dev, serial)) = USB_STATE.borrow_ref_mut(cs).as_mut() {
                    // Polling here (not only in the IRQ) lets a caller at any
                    // priority drain the buffer itself instead of waiting for
                    // a lower-priority IRQ that cannot preempt it.
                    usb_dev.poll(&mut [serial]);

                    match serial.write(&buf[written..]) {
                        Ok(n) if n > 0 => {
                            written += n;
                            idle_polls = 0;
                        }
                        Ok(_) | Err(UsbError::WouldBlock) => idle_polls += 1,
                        Err(_) => give_up = true,
                    }
                } else {
                    give_up = true;
                }
            });
            if give_up {
                break;
            }
        }

        Ok(if written == 0 { buf.len() } else { written })
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub fn poll() {
    critical_section::with(|cs| {
        if let Some((usb_dev, serial)) = USB_STATE.borrow_ref_mut(cs).as_mut() {
            usb_dev.poll(&mut [serial]);
        }
    })
}

pub fn init(
    regs: pac::USBCTRL_REGS,
    dpram: pac::USBCTRL_DPRAM,
    usb_clock: UsbClock,
    resets: &mut pac::RESETS,
) {
    let usb_bus: &'static UsbBusAllocator<UsbBus> = USB_BUS.init(UsbBusAllocator::new(
        UsbBus::new(regs, dpram, usb_clock, true, resets),
    ));

    let serial = SerialPort::new(usb_bus);
    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::new(LangID::EN).product("srf05-imu-station")])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    critical_section::with(|cs| {
        *USB_STATE.borrow_ref_mut(cs) = Some((usb_dev, serial));
    });

    defmt_serial::defmt_serial(WRITER.init(DefmtUsbWriter));
}
