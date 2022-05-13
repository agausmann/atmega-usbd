#![no_std]

use core::cmp::max;

use atmega_hal::pac::USB_DEVICE;
use avr_device::interrupt::{self, CriticalSection, Mutex};
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
    is_allocated: bool,
    eptype_bits: u8,
    epdir_bit: bool,
    epsize_bits: u8,
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

    fn set_current_endpoint(&self, cs: &CriticalSection, index: usize) -> Result<(), UsbError> {
        if index >= MAX_ENDPOINTS {
            return Err(UsbError::InvalidEndpoint);
        }
        self.usb
            .borrow(cs)
            .uenum
            .write(|w| unsafe { w.bits(index as u8) });
        Ok(())
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
            Some(addr) if !self.endpoints[addr.index()].is_allocated => addr,
            _ => {
                // Find next free endpoint
                let index = self
                    .endpoints
                    .iter()
                    .enumerate()
                    .find_map(|(index, ep)| {
                        if !ep.is_allocated && max_packet_size <= ENDPOINT_MAX_BUFSIZE[index] {
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
        entry.eptype_bits = match ep_type {
            EndpointType::Control => EP_TYPE_CONTROL,
            EndpointType::Isochronous => EP_TYPE_ISOCHRONOUS,
            EndpointType::Bulk => EP_TYPE_BULK,
            EndpointType::Interrupt => EP_TYPE_INTERRUPT,
        };
        entry.epdir_bit = match ep_dir {
            UsbDirection::Out => EP_DIR_OUT,
            UsbDirection::In => EP_DIR_IN,
        };
        let ep_size = max(8, max_packet_size.next_power_of_two());
        if DPRAM_SIZE - self.dpram_usage < ep_size {
            return Err(UsbError::EndpointMemoryOverflow);
        }
        entry.epsize_bits = match ep_size {
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
        entry.is_allocated = true;
        self.dpram_usage += ep_size;
        Ok(ep_addr)
    }

    fn enable(&mut self) {
        interrupt::free(|cs| {
            let usb = self.usb.borrow(cs);
            usb.uhwcon.modify(|_, w| w.uvrege().set_bit());
            usb.usbcon
                .modify(|_, w| w.usbe().set_bit().otgpade().set_bit());
            // NB: FRZCLK must be modified while USBE is set:
            usb.usbcon.modify(|_, w| w.frzclk().clear_bit());
            usb.udcon.modify(|_, w| w.detach().clear_bit());
        });
    }

    fn reset(&self) {
        interrupt::free(|cs| {
            let usb = self.usb.borrow(cs);
            usb.udint.modify(|_, w| w.eorsti().clear_bit());

            for (index, endpoint) in self.endpoints.iter().enumerate() {
                if !endpoint.is_allocated {
                    continue;
                }

                self.set_current_endpoint(cs, index).unwrap();
                usb.ueconx.modify(|_, w| w.epen().set_bit());

                if usb.uecfg1x.read().alloc().bit_is_set() {
                    continue;
                }

                usb.uecfg0x.write(|w| {
                    w.epdir()
                        .bit(endpoint.epdir_bit)
                        .eptype()
                        .bits(endpoint.eptype_bits)
                });
                usb.uecfg1x
                    .write(|w| w.epbk().bits(0).epsize().bits(endpoint.epsize_bits));
                usb.uecfg1x.modify(|_, w| w.alloc().set_bit());

                assert!(
                    usb.uesta0x.read().cfgok().bit_is_set(),
                    "could not configure endpoint {}",
                    index
                );
            }
        })
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
