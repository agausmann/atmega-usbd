#![no_std]

use atmega_hal::pac::USB_DEVICE;
use avr_device::interrupt::Mutex;

const MAX_ENDPOINTS: usize = 7;
const ENDPOINT_MAX_BUFSIZE: [u16; MAX_ENDPOINTS] = [64, 256, 64, 64, 64, 64, 64];
const DPRAM_SIZE: u16 = 832;

const EP_TYPE_CONTROL: u8 = 0b00;
const EP_TYPE_ISOCHRONOUS: u8 = 0b01;
const EP_TYPE_BULK: u8 = 0b10;
const EP_TYPE_INTERRUPT: u8 = 0b11;

const EP_DIR_IN: bool = true;
const EP_DIR_OUT: bool = false;

const EP_SIZE_8: u8 = 0b000;
const EP_SIZE_16: u8 = 0b001;
const EP_SIZE_32: u8 = 0b010;
const EP_SIZE_64: u8 = 0b011;
const EP_SIZE_128: u8 = 0b100;
const EP_SIZE_256: u8 = 0b101;
const EP_SIZE_512: u8 = 0b110;

#[derive(Default)]
struct EndpointTableEntry {
    allocated: bool,
    ep_type_bits: u8,
    ep_dir_bit: bool,
    ep_size_bits: u8,
}

pub struct UsbBus {
    usb: Mutex<USB_DEVICE>,
    endpoints: [EndpointTableEntry; MAX_ENDPOINTS],
}

impl UsbBus {
    pub fn new(usb: USB_DEVICE) -> Self {
        Self {
            usb: Mutex::new(usb),
            endpoints: Default::default(),
        }
    }
}

impl usb_device::bus::UsbBus for UsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: usb_device::UsbDirection,
        ep_addr: Option<usb_device::class_prelude::EndpointAddress>,
        ep_type: usb_device::class_prelude::EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> usb_device::Result<usb_device::class_prelude::EndpointAddress> {
        todo!()
    }

    fn enable(&mut self) {
        todo!()
    }

    fn reset(&self) {
        todo!()
    }

    fn set_device_address(&self, addr: u8) {
        todo!()
    }

    fn write(
        &self,
        ep_addr: usb_device::class_prelude::EndpointAddress,
        buf: &[u8],
    ) -> usb_device::Result<usize> {
        todo!()
    }

    fn read(
        &self,
        ep_addr: usb_device::class_prelude::EndpointAddress,
        buf: &mut [u8],
    ) -> usb_device::Result<usize> {
        todo!()
    }

    fn set_stalled(&self, ep_addr: usb_device::class_prelude::EndpointAddress, stalled: bool) {
        todo!()
    }

    fn is_stalled(&self, ep_addr: usb_device::class_prelude::EndpointAddress) -> bool {
        todo!()
    }

    fn suspend(&self) {
        todo!()
    }

    fn resume(&self) {
        todo!()
    }

    fn poll(&self) -> usb_device::bus::PollResult {
        todo!()
    }
}
