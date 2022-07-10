#![no_std]
#![cfg_attr(not(test), no_main)]
#![feature(lang_items)]
#![feature(abi_avr_interrupt)]

mod std_stub;

use arduino_hal::{pins, Peripherals};
use atmega_usbd::UsbBus;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::HIDClass,
};

#[no_mangle]
pub extern "C" fn main() {
    main_inner();
}

fn main_inner() {
    let dp = Peripherals::take().unwrap();
    let pins = pins!(dp);
    let pll = dp.PLL;
    let usb = dp.USB_DEVICE;

    let mut status = pins.d13.into_output();
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

    let usb_bus = UsbBus::new(usb);
    let mut hid_class = HIDClass::new(&usb_bus, KeyboardReport::desc(), 1);
    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .manufacturer("Foo")
        .product("Bar")
        .build();

    let mut report_buf = [0u8; 1];
    let mut index = 0;

    loop {
        if trigger.is_low() {
            if let Some(report) = PAYLOAD.get(index).copied().and_then(ascii_to_report) {
                if hid_class.push_input(&report).is_ok() {
                    index += 1;
                }
            } else {
                hid_class
                    .push_input(&KeyboardReport {
                        modifier: 0,
                        reserved: 0,
                        leds: 0,
                        keycodes: [0; 6],
                    })
                    .ok();
            }
        } else {
            index = 0;
            hid_class
                .push_input(&KeyboardReport {
                    modifier: 0,
                    reserved: 0,
                    leds: 0,
                    keycodes: [0; 6],
                })
                .ok();
        }

        if usb_device.poll(&mut [&mut hid_class]) {
            if hid_class.pull_raw_output(&mut report_buf).is_ok() {
                if report_buf[0] & 2 != 0 {
                    status.set_high();
                } else {
                    status.set_low();
                }
            }
        }
    }
}

const PAYLOAD: &[u8] = b"Hello World";

fn ascii_to_report(c: u8) -> Option<KeyboardReport> {
    let (keycode, shift) = if c.is_ascii_alphabetic() {
        (c.to_ascii_lowercase() - b'a' + 0x04, c.is_ascii_uppercase())
    } else {
        match c {
            b' ' => (0x2c, false),
            _ => return None,
        }
    };

    Some(KeyboardReport {
        modifier: if shift { 2 } else { 0 },
        reserved: 0,
        leds: 0,
        keycodes: [keycode, 0, 0, 0, 0, 0],
    })
}
