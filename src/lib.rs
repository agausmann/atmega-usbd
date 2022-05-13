#![no_std]

use core::cmp::max;

use atmega_hal::pac::USB_DEVICE;
use avr_device::interrupt::Mutex;
use usb_device::{
    endpoint::{EndpointAddress, EndpointType},
    UsbDirection, UsbError,
};

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
    dpram_usage: u16,
}

impl UsbBus {
    pub fn new(usb: USB_DEVICE) -> Self {
        Self {
            usb: Mutex::new(usb),
            endpoints: Default::default(),
            dpram_usage: 0,
        }
    }
}

impl usb_device::bus::UsbBus for UsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> Result<EndpointAddress, UsbError> {
        // Ignore duplicate ep0 allocation by usb_device.
        // Endpoints can only be configured once, and
        // control endpoint must be configured as "OUT".
        if ep_addr == Some(EndpointAddress::from_parts(0, UsbDirection::In)) {
            return Ok(ep_addr.unwrap());
        }

        let ep_addr = match ep_addr {
            Some(addr) if !self.endpoints[addr.index()].allocated => addr,
            _ => {
                // Find next free endpoint
                let index = self
                    .endpoints
                    .iter()
                    .enumerate()
                    .find_map(|(index, ep)| {
                        if !ep.allocated && max_packet_size <= ENDPOINT_MAX_BUFSIZE[index] {
                            Some(index)
                        } else {
                            None
                        }
                    })
                    .ok_or(UsbError::EndpointOverflow)?;
                EndpointAddress::from_parts(index, ep_dir)
            }
        };
        let entry = &mut self.endpoints[ep_addr.index()];
        entry.ep_type_bits = match ep_type {
            EndpointType::Control => EP_TYPE_CONTROL,
            EndpointType::Isochronous => EP_TYPE_ISOCHRONOUS,
            EndpointType::Bulk => EP_TYPE_BULK,
            EndpointType::Interrupt => EP_TYPE_INTERRUPT,
        };
        entry.ep_dir_bit = match ep_dir {
            UsbDirection::Out => EP_DIR_OUT,
            UsbDirection::In => EP_DIR_IN,
        };
        let ep_size = max(8, max_packet_size.next_power_of_two());
        if DPRAM_SIZE - self.dpram_usage < ep_size {
            return Err(UsbError::EndpointMemoryOverflow);
        }
        entry.ep_size_bits = match ep_size {
            8 => EP_SIZE_8,
            16 => EP_SIZE_16,
            32 => EP_SIZE_32,
            64 => EP_SIZE_64,
            128 => EP_SIZE_128,
            256 => EP_SIZE_256,
            512 => EP_SIZE_512,
            _ => return Err(UsbError::EndpointMemoryOverflow),
        };

        // Configuration succeeded, commit/finalize:
        entry.allocated = true;
        self.dpram_usage += ep_size;
        Ok(ep_addr)
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

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
        todo!()
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
        todo!()
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        todo!()
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
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
