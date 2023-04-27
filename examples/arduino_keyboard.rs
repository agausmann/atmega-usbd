#![no_std]
#![cfg_attr(not(test), no_main)]
#![feature(lang_items)]
#![feature(abi_avr_interrupt)]
#![deny(unsafe_op_in_unsafe_fn)]

mod std_stub;

use arduino_hal::{
    entry,
    pac::PLL,
    pins,
    port::{
        mode::{Input, Output, PullUp},
        Pin,
    },
    Peripherals,
};
use atmega_usbd::{SuspendNotifier, UsbBus};
use avr_device::{asm::sleep, interrupt};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::HIDClass,
};

const PAYLOAD: &[u8] = b"Hello World";

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let pins = pins!(dp);
    let pll = dp.PLL;
    let usb = dp.USB_DEVICE;

    let status = pins.d13.into_output();
    let trigger = pins.d2.into_pull_up_input();

    // Configure PLL interface
    // prescale 16MHz crystal -> 8MHz
    pll.pllcsr.write(|w| w.pindiv().set_bit());
    // 96MHz PLL output; /1.5 for 64MHz timers, /2 for 48MHz USB
    pll.pllfrq
        .write(|w| w.pdiv().mhz96().plltm().factor_15().pllusb().set_bit());

    // Enable PLL
    pll.pllcsr.modify(|_, w| w.plle().set_bit());

    // Check PLL lock
    while pll.pllcsr.read().plock().bit_is_clear() {}

    let usb_bus = unsafe {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<PLL>>> = None;
        &*USB_BUS.insert(UsbBus::new_with_notifier(usb, pll))
    };

    let hid_class = HIDClass::new(&usb_bus, KeyboardReport::desc(), 1);
    let usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .manufacturer("Foo")
        .product("Bar")
        .build();

    unsafe {
        USB_CTX = Some(UsbContext {
            usb_device,
            hid_class,
            current_index: 0,
            indicator: status.downgrade(),
            trigger: trigger.downgrade(),
        });
    }

    unsafe { interrupt::enable() };
    loop {
        sleep();
    }
}

static mut USB_CTX: Option<UsbContext<PLL>> = None;

#[interrupt(atmega32u4)]
fn USB_GEN() {
    unsafe { poll_usb() };
}

#[interrupt(atmega32u4)]
fn USB_COM() {
    unsafe { poll_usb() };
}

/// # Safety
///
/// This function assumes that it is being called within an
/// interrupt context.
unsafe fn poll_usb() {
    // Safety: There must be no other overlapping borrows of USB_CTX.
    // - By the safety contract of this function, we are in an interrupt
    //   context.
    // - The main thread is not borrowing USB_CTX. The only access is the
    //   assignment during initialization. It cannot overlap because it is
    //   before the call to `interrupt::enable()`.
    // - No other interrupts are accessing USB_CTX, because no other interrupts
    //   are in the middle of execution. GIE is automatically cleared for the
    //   duration of the interrupt, and is not re-enabled within any ISRs.
    let ctx = unsafe { USB_CTX.as_mut().unwrap() };
    ctx.poll();
}

struct UsbContext<S: SuspendNotifier> {
    usb_device: UsbDevice<'static, UsbBus<S>>,
    hid_class: HIDClass<'static, UsbBus<S>>,
    current_index: usize,
    indicator: Pin<Output>,
    trigger: Pin<Input<PullUp>>,
}

impl<S: SuspendNotifier> UsbContext<S> {
    fn poll(&mut self) {
        if self.trigger.is_low() {
            if let Some(report) = PAYLOAD
                .get(self.current_index)
                .copied()
                .and_then(ascii_to_report)
            {
                if self.hid_class.push_input(&report).is_ok() {
                    self.current_index += 1;
                }
            } else {
                self.hid_class.push_input(&BLANK_REPORT).ok();
            }
        } else {
            self.current_index = 0;
            self.hid_class.push_input(&BLANK_REPORT).ok();
        }

        if self.usb_device.poll(&mut [&mut self.hid_class]) {
            let mut report_buf = [0u8; 1];

            if self.hid_class.pull_raw_output(&mut report_buf).is_ok() {
                if report_buf[0] & 2 != 0 {
                    self.indicator.set_high();
                } else {
                    self.indicator.set_low();
                }
            }
        }
    }
}

const BLANK_REPORT: KeyboardReport = KeyboardReport {
    modifier: 0,
    reserved: 0,
    leds: 0,
    keycodes: [0; 6],
};

fn ascii_to_report(c: u8) -> Option<KeyboardReport> {
    let (keycode, shift) = if c.is_ascii_alphabetic() {
        (c.to_ascii_lowercase() - b'a' + 0x04, c.is_ascii_uppercase())
    } else {
        match c {
            b' ' => (0x2c, false),
            _ => return None,
        }
    };

    let mut report = BLANK_REPORT;
    if shift {
        report.modifier |= 0x2;
    }
    report.keycodes[0] = keycode;
    Some(report)
}
