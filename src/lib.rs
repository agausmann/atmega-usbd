#![no_std]
#![feature(asm_experimental_arch)]

use core::{arch::asm, cell::Cell, cmp::max};

use avr_device::atmega32u4::{
    usb_device::{udint, ueintx, usbint, UDINT, UEINTX, USBINT},
    PLL, USB_DEVICE,
};
use avr_device::interrupt::{self, CriticalSection, Mutex};
use usb_device::{
    bus::PollResult,
    class_prelude::UsbBusAllocator,
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

impl EndpointTableEntry {
    fn buffer_size(&self) -> usize {
        match self.epsize_bits {
            EP_SIZE_8 => 8,
            EP_SIZE_16 => 16,
            EP_SIZE_32 => 32,
            EP_SIZE_64 => 64,
            EP_SIZE_128 => 128,
            EP_SIZE_256 => 256,
            EP_SIZE_512 => 512,
            _ => unreachable!(),
        }
    }
}

pub struct UsbBus<S: SuspendNotifier> {
    usb: Mutex<USB_DEVICE>,
    suspend_notifier: Mutex<S>,
    pending_ins: Mutex<Cell<u8>>,
    endpoints: [EndpointTableEntry; MAX_ENDPOINTS],
    dpram_usage: u16,
}

impl UsbBus<()> {
    /// Create a new UsbBus without power-saving functionality.
    ///
    /// If you would like to disable the PLL when the USB peripheral is
    /// suspended, then construct the bus with [`UsbBus::with_suspend_notifier`].
    pub fn new(usb: USB_DEVICE) -> UsbBusAllocator<Self> {
        Self::with_suspend_notifier(usb, ())
    }
}

impl<S: SuspendNotifier> UsbBus<S> {
    /// Create a UsbBus with a suspend and resume handler.
    ///
    /// If you want the PLL to be automatically disabled when the USB peripheral
    /// is suspended, then you can pass the PLL resource here; for example:
    ///
    /// ```
    /// use avr_device::atmega32u4::Peripherals;
    /// use atmega_usbd::UsbBus;
    ///
    /// let dp = Peripherals.take().unwrap();
    /// // ... (other initialization stuff)
    /// let bus = UsbBus::with_suspend_notifier(dp.USB_DEVICE, dp.PLL);
    /// ```
    ///
    /// **Note: If you are using the PLL output for other peripherals like the
    /// high-speed timer, then disabling the PLL may affect the behavior of
    /// those peripherals.** In such cases, you can either use [`UsbBus::new`]
    /// to leave the PLL running, or implement [`SuspendNotifier`] yourself,
    /// with some custom logic to gracefully shut down the PLL in cooperation
    /// with your other peripherals.
    pub fn with_suspend_notifier(usb: USB_DEVICE, suspend_notifier: S) -> UsbBusAllocator<Self> {
        UsbBusAllocator::new(Self {
            usb: Mutex::new(usb),
            suspend_notifier: Mutex::new(suspend_notifier),
            pending_ins: Mutex::new(Cell::new(0)),
            endpoints: Default::default(),
            dpram_usage: 0,
        })
    }

    fn active_endpoints(&self) -> impl Iterator<Item = (usize, &EndpointTableEntry)> {
        self.endpoints
            .iter()
            .enumerate()
            .filter(|&(_, ep)| ep.is_allocated)
    }

    fn set_current_endpoint(&self, cs: CriticalSection, index: usize) -> Result<(), UsbError> {
        if index >= MAX_ENDPOINTS {
            return Err(UsbError::InvalidEndpoint);
        }
        let usb = self.usb.borrow(cs);
        if usb.usbcon.read().frzclk().bit_is_set() {
            return Err(UsbError::InvalidState);
        }
        usb.uenum.write(|w| w.bits(index as u8));
        if usb.uenum.read().bits() & 0b111 != (index as u8) {
            return Err(UsbError::InvalidState);
        }
        Ok(())
    }

    fn endpoint_byte_count(&self, cs: CriticalSection) -> u16 {
        let usb = self.usb.borrow(cs);
        // FIXME: Potential for desync here? LUFA doesn't seem to care.
        ((usb.uebchx.read().bits() as u16) << 8) | (usb.uebclx.read().bits() as u16)
    }

    fn configure_endpoint(&self, cs: CriticalSection, index: usize) -> Result<(), UsbError> {
        let usb = self.usb.borrow(cs);
        self.set_current_endpoint(cs, index)?;
        let endpoint = &self.endpoints[index];

        usb.ueconx.modify(|_, w| w.epen().set_bit());
        usb.uecfg1x.modify(|_, w| w.alloc().clear_bit());

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

        usb.ueienx
            .modify(|_, w| w.rxoute().set_bit().rxstpe().set_bit());
        Ok(())
    }
}

impl<S: SuspendNotifier> usb_device::bus::UsbBus for UsbBus<S> {
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
                    .skip(1)
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
            EndpointType::Isochronous { .. } => EP_TYPE_ISOCHRONOUS,
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
            // NB: FRZCLK cannot be set/cleared when USBE=0, and
            // cannot be modified at the same time.
            usb.usbcon
                .modify(|_, w| w.frzclk().clear_bit().vbuste().set_bit());

            for (index, _ep) in self.active_endpoints() {
                self.configure_endpoint(cs, index).unwrap();
            }

            usb.udcon.modify(|_, w| w.detach().clear_bit());
            usb.udien
                .modify(|_, w| w.eorste().set_bit().sofe().set_bit());
        });
    }

    fn reset(&self) {
        interrupt::free(|cs| {
            let usb = self.usb.borrow(cs);
            usb.udint.modify(|_, w| w.eorsti().clear_bit());

            for (index, _ep) in self.active_endpoints() {
                self.configure_endpoint(cs, index).unwrap();
            }

            usb.udint
                .clear_interrupts(|w| w.wakeupi().clear_bit().suspi().clear_bit());
            usb.udien
                .modify(|_, w| w.wakeupe().clear_bit().suspe().set_bit());
        })
    }

    fn set_device_address(&self, addr: u8) {
        interrupt::free(|cs| {
            let usb = self.usb.borrow(cs);
            usb.udaddr.modify(|_, w| w.uadd().bits(addr));
            // NB: ADDEN and UADD shall not be written at the same time.
            usb.udaddr.modify(|_, w| w.adden().set_bit());
        });
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
        interrupt::free(|cs| {
            let usb = self.usb.borrow(cs);
            self.set_current_endpoint(cs, ep_addr.index())?;
            let endpoint = &self.endpoints[ep_addr.index()];

            // Different logic is needed for control endpoints:
            // - The FIFOCON and RWAL fields are irrelevant with CONTROL endpoints.
            // - TXINI ... shall be cleared by firmware to **send the
            //   packet and to clear the endpoint bank.**
            if endpoint.eptype_bits == EP_TYPE_CONTROL {
                if usb.ueintx.read().txini().bit_is_clear() {
                    return Err(UsbError::WouldBlock);
                }

                let buffer_size = endpoint.buffer_size();
                if buf.len() > buffer_size {
                    return Err(UsbError::BufferOverflow);
                }

                for &byte in buf {
                    usb.uedatx.write(|w| w.bits(byte))
                }

                usb.ueintx.clear_interrupts(|w| w.txini().clear_bit());
            } else {
                if usb.ueintx.read().txini().bit_is_clear() {
                    return Err(UsbError::WouldBlock);
                }
                //NB: RXOUTI serves as KILLBK for IN endpoints and needs to stay zero:
                usb.ueintx
                    .clear_interrupts(|w| w.txini().clear_bit().rxouti().clear_bit());

                for &byte in buf {
                    if usb.ueintx.read().rwal().bit_is_clear() {
                        return Err(UsbError::BufferOverflow);
                    }
                    usb.uedatx.write(|w| w.bits(byte));
                }

                //NB: RXOUTI serves as KILLBK for IN endpoints and needs to stay zero:
                usb.ueintx
                    .clear_interrupts(|w| w.fifocon().clear_bit().rxouti().clear_bit());
            }

            let pending_ins = self.pending_ins.borrow(cs);
            pending_ins.set(pending_ins.get() | 1 << ep_addr.index());

            Ok(buf.len())
        })
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
        interrupt::free(|cs| {
            let usb = self.usb.borrow(cs);
            self.set_current_endpoint(cs, ep_addr.index())?;
            let endpoint = &self.endpoints[ep_addr.index()];

            // Different logic is needed for control endpoints:
            // - The FIFOCON and RWAL fields are irrelevant with CONTROL endpoints.
            // - RXSTPI/RXOUTI ... shall be cleared by firmware to **send the
            //   packet and to clear the endpoint bank.**
            if endpoint.eptype_bits == EP_TYPE_CONTROL {
                let ueintx = usb.ueintx.read();
                if ueintx.rxouti().bit_is_clear() && ueintx.rxstpi().bit_is_clear() {
                    return Err(UsbError::WouldBlock);
                }

                let bytes_to_read = self.endpoint_byte_count(cs) as usize;
                if bytes_to_read > buf.len() {
                    return Err(UsbError::BufferOverflow);
                }

                for slot in &mut buf[..bytes_to_read] {
                    *slot = usb.uedatx.read().bits();
                }
                usb.ueintx
                    .clear_interrupts(|w| w.rxouti().clear_bit().rxstpi().clear_bit());

                Ok(bytes_to_read)
            } else {
                if usb.ueintx.read().rxouti().bit_is_clear() {
                    return Err(UsbError::WouldBlock);
                }
                usb.ueintx.clear_interrupts(|w| w.rxouti().clear_bit());

                let mut bytes_read = 0;
                for slot in buf {
                    if usb.ueintx.read().rwal().bit_is_clear() {
                        break;
                    }
                    *slot = usb.uedatx.read().bits();
                    bytes_read += 1;
                }

                if usb.ueintx.read().rwal().bit_is_set() {
                    return Err(UsbError::BufferOverflow);
                }

                usb.ueintx.clear_interrupts(|w| w.fifocon().clear_bit());
                Ok(bytes_read)
            }
        })
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        interrupt::free(|cs| {
            let usb = self.usb.borrow(cs);
            if self.set_current_endpoint(cs, ep_addr.index()).is_ok() {
                usb.ueconx
                    .modify(|_, w| w.stallrq().bit(stalled).stallrqc().bit(!stalled));
            }
        });
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        interrupt::free(|cs| {
            let usb = self.usb.borrow(cs);
            if self.set_current_endpoint(cs, ep_addr.index()).is_ok() {
                // NB: The datasheet says STALLRQ is write-only but LUFA reads from it...
                usb.ueconx.read().stallrq().bit_is_set()
            } else {
                false
            }
        })
    }

    fn suspend(&self) {
        interrupt::free(|cs| {
            let usb = self.usb.borrow(cs);
            usb.udint
                .clear_interrupts(|w| w.suspi().clear_bit().wakeupi().clear_bit());
            usb.udien
                .modify(|_, w| w.wakeupe().set_bit().suspe().clear_bit());
            usb.usbcon.modify(|_, w| w.frzclk().set_bit());

            self.suspend_notifier.borrow(cs).suspend();
        });
    }

    fn resume(&self) {
        interrupt::free(|cs| {
            self.suspend_notifier.borrow(cs).resume();

            let usb = self.usb.borrow(cs);
            usb.usbcon.modify(|_, w| w.frzclk().clear_bit());
            usb.udint
                .clear_interrupts(|w| w.wakeupi().clear_bit().suspi().clear_bit());
            usb.udien
                .modify(|_, w| w.wakeupe().clear_bit().suspe().set_bit());
        });
    }

    fn poll(&self) -> PollResult {
        interrupt::free(|cs| {
            let usb = self.usb.borrow(cs);

            let usbint = usb.usbint.read();
            let udint = usb.udint.read();
            let udien = usb.udien.read();
            if usbint.vbusti().bit_is_set() {
                usb.usbint.clear_interrupts(|w| w.vbusti().clear_bit());
                if usb.usbsta.read().vbus().bit_is_set() {
                    return PollResult::Resume;
                } else {
                    return PollResult::Suspend;
                }
            }
            if udint.suspi().bit_is_set() && udien.suspe().bit_is_set() {
                return PollResult::Suspend;
            }
            if udint.wakeupi().bit_is_set() && udien.wakeupe().bit_is_set() {
                return PollResult::Resume;
            }
            if udint.eorsti().bit_is_set() {
                return PollResult::Reset;
            }
            if udint.sofi().bit_is_set() {
                usb.udint.clear_interrupts(|w| w.sofi().clear_bit());
            }

            // Can only query endpoints while clock is running
            // (e.g. not in suspend state)
            if usb.usbcon.read().frzclk().bit_is_clear() {
                let mut ep_out = 0u8;
                let mut ep_setup = 0u8;
                let mut ep_in_complete = 0u8;
                let pending_ins = self.pending_ins.borrow(cs);

                for (index, _ep) in self.active_endpoints() {
                    if self.set_current_endpoint(cs, index).is_err() {
                        // Endpoint selection has stopped working...
                        break;
                    }

                    let ueintx = usb.ueintx.read();
                    if ueintx.rxouti().bit_is_set() {
                        ep_out |= 1 << index;
                    }
                    if ueintx.rxstpi().bit_is_set() {
                        ep_setup |= 1 << index;
                    }
                    if pending_ins.get() & (1 << index) != 0 && ueintx.txini().bit_is_set() {
                        ep_in_complete |= 1 << index;
                        pending_ins.set(pending_ins.get() & !(1 << index));
                    }
                }
                if ep_out | ep_setup | ep_in_complete != 0 {
                    return PollResult::Data {
                        ep_out: ep_out as u16,
                        ep_in_complete: ep_in_complete as u16,
                        ep_setup: ep_setup as u16,
                    };
                }
            }

            PollResult::None
        })
    }

    fn force_reset(&self) -> usb_device::Result<()> {
        // 22.9 "It is possible to re-enumerate a device, simply by setting and
        // clearing the DETACH bit (but firmware must take in account a
        // debouncing delay of some milliseconds)."

        interrupt::free(|cs| {
            self.usb
                .borrow(cs)
                .udcon
                .modify(|_, w| w.detach().set_bit());
        });

        // Delay for at least 1ms (exactly 1ms at 16 MHz)
        // to allow the host to detect the change.
        delay_cycles(16000);

        interrupt::free(|cs| {
            self.usb
                .borrow(cs)
                .udcon
                .modify(|_, w| w.detach().clear_bit());
        });

        Ok(())
    }
}

/// Extension trait for conveniently clearing AVR interrupt flag registers.
///
/// To clear an interrupt flag, a zero bit must be written. However, there are
/// several other hazards to take into consideration:
///
/// 1. If you read-modify-write, it is possible that an interrupt flag will be
///   set by hardware in between the read and write, and writing the zero that
///   you previously read will clear that flag as well. So, use a default value
///   of all ones and specifically clear the bits you want. HOWEVER:
///
/// 2. Some bits of the interrupt flag register are reserved, and it is
///   specified that they should not be written as ones.
///
/// Implementers of this trait should provide an initial value to the callback
/// with all _known_ interrupt flags set to the value that has no effect (which
/// is 1, in most cases)
trait ClearInterrupts {
    type Writer;

    fn clear_interrupts<F>(&self, f: F)
    where
        for<'w> F: FnOnce(&mut Self::Writer) -> &mut Self::Writer;
}

impl ClearInterrupts for UDINT {
    type Writer = udint::W;

    fn clear_interrupts<F>(&self, f: F)
    where
        for<'w> F: FnOnce(&mut Self::Writer) -> &mut Self::Writer,
    {
        // Bits 1,7 reserved as do not set. Setting all other bits has no effect
        self.write(|w| f(unsafe { w.bits(0x7d) }))
    }
}

impl ClearInterrupts for UEINTX {
    type Writer = ueintx::W;

    fn clear_interrupts<F>(&self, f: F)
    where
        for<'w> F: FnOnce(&mut Self::Writer) -> &mut Self::Writer,
    {
        // Bit 5 read-only. Setting all other bits has no effect, EXCEPT:
        //  - RXOUTI/KILLBK should not be set for "IN" endpoints (XXX end-user beware)
        self.write(|w| f(unsafe { w.bits(0xdf) }))
    }
}

impl ClearInterrupts for USBINT {
    type Writer = usbint::W;

    fn clear_interrupts<F>(&self, f: F)
    where
        for<'w> F: FnOnce(&mut Self::Writer) -> &mut Self::Writer,
    {
        // Bits 7:1 are reserved as do not set.
        self.write(|w| f(unsafe { w.bits(0x01) }))
    }
}

/// Receiver for handling suspend and resume events from the USB device.
///
/// See [`UsbBus::with_suspend_notifier`] for more details.
pub trait SuspendNotifier: Send + Sized + 'static {
    /// Called by `UsbBus` when the USB peripheral has been suspended and the
    /// PLL is safe to shut down.
    fn suspend(&self) {}

    /// Called by `UsbBus` when the USB peripheral is about to resume and is
    /// waiting for PLL to be enabled.
    ///
    /// This function should block until PLL lock has been established.
    fn resume(&self) {}
}

impl SuspendNotifier for () {}

impl SuspendNotifier for PLL {
    fn suspend(&self) {
        self.pllcsr.modify(|_, w| w.plle().clear_bit());
    }

    fn resume(&self) {
        self.pllcsr
            .modify(|_, w| w.pindiv().set_bit().plle().set_bit());

        while self.pllcsr.read().plock().bit_is_clear() {}
    }
}

/// Placeholder for `avr_device::asm::delay_cycles`
///
/// https://github.com/Rahix/avr-device/pull/127
#[inline(always)]
fn delay_cycles(cycles: u32) {
    let mut cycles_bytes = cycles.to_le_bytes();
    // Each loop iteration takes 6 cycles when the branch is taken,
    // and 5 cycles when the branch is not taken.
    // So, this loop is guaranteed to run for at least `cycles - 1` cycles,
    // and there will be approximately 4 cycles before the loop to initialize
    // the counting registers.
    unsafe {
        asm!(
            "1:",
            "subi {r0}, 6",
            "sbci {r1}, 0",
            "sbci {r2}, 0",
            "sbci {r3}, 0",
            "brcc 1b",

            r0 = inout(reg_upper) cycles_bytes[0],
            r1 = inout(reg_upper) cycles_bytes[1],
            r2 = inout(reg_upper) cycles_bytes[2],
            r3 = inout(reg_upper) cycles_bytes[3],
        )
    }
}
