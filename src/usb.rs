use dap_rs::usb::{dap_v1::CmsisDapV1, dap_v2::CmsisDapV2, winusb::MicrosoftDescriptors, Request};
use dap_rs::usb_device::{
    bus::UsbBus as UsbBusTrait, class_prelude::*, prelude::*, LangID, Result as UsbResult,
};
use defmt::*;
use rp2040_hal::usb::UsbBus;
use usbd_serial::SerialPort;

pub struct DebugInterface<'a, B: UsbBusTrait> {
    interface: InterfaceNumber,
    name_index: StringIndex,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,

    #[cfg(feature = "defmt-bbq")]
    defmt_consumer: defmt_brtt::DefmtConsumer,
    #[cfg(feature = "defmt-bbq")]
    last_packet_size: usize,
}

impl<'a, B: UsbBusTrait> DebugInterface<'a, B> {
    pub fn new(
        alloc: &'a UsbBusAllocator<B>,
        packet_size: u16,
        #[cfg(feature = "defmt-bbq")] defmt_consumer: defmt_brtt::DefmtConsumer,
    ) -> Self {
        Self {
            interface: alloc.interface(),
            name_index: alloc.string(),
            read_ep: alloc.bulk(packet_size),
            write_ep: alloc.bulk(packet_size),
            #[cfg(feature = "defmt-bbq")]
            defmt_consumer,
            #[cfg(feature = "defmt-bbq")]
            last_packet_size: 0,
        }
    }

    pub fn write(&mut self, data: &[u8]) -> UsbResult<usize> {
        self.write_ep.write(data)
    }

    pub fn read(&mut self, data: &mut [u8]) -> UsbResult<usize> {
        self.read_ep.read(data)
    }

    #[cfg(feature = "defmt-bbq")]
    pub fn pump_consumer(&mut self) {
        match self.defmt_consumer.read() {
            Ok(grant) => {
                let usb_packet = if grant.len() > self.write_ep.max_packet_size().into() {
                    &grant[0..usize::from(self.write_ep.max_packet_size())]
                } else {
                    &grant
                };
                // fixme: are we doing packets right?
                let bytes_written = if let Ok(bytes_written) = self.write_ep.write(usb_packet) {
                    self.last_packet_size = bytes_written;
                    bytes_written
                } else {
                    0
                };
                grant.release(bytes_written);
            }
            Err(defmt_brtt::BBQError::Bbq(bbqueue::Error::InsufficientSize)) => {
                // When we have no more defmt data, we might still need a ZLP to
                // signal to the USB host that we're done.  "Done" means a packet
                // whose size is less than the endpoint's packet size.
                if self.last_packet_size == self.write_ep.max_packet_size().into() {
                    let _ = self.write_ep.write(&[]);
                }
            }
            Err(defmt_brtt::BBQError::Bbq(_)) => {
                defmt::unreachable!();
            }
            Err(_) => {
                defmt::unreachable!();
            }
        }
    }
}

impl<B: UsbBusTrait> UsbClass<B> for DebugInterface<'_, B> {
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> UsbResult<()> {
        // altsetting 0; default.  We use interface_alt to pass the string descriptor.
        // 0xff: vendor-specific class; subclass 0, protocol 0
        writer.interface_alt(self.interface, 0, 0xff, 0x00, 0x00, Some(self.name_index))?;

        writer.endpoint(&self.read_ep)?;
        writer.endpoint(&self.write_ep)?;

        Ok(())
    }

    fn get_string(&self, index: StringIndex, _langid: LangID) -> Option<&str> {
        if index == self.name_index {
            Some("Rusty-Probe Debug Interface")
        } else {
            None
        }
    }

    // fn reset(&mut self) {}

    #[cfg(feature = "defmt-bbq")]
    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.write_ep.address() {
            self.pump_consumer();
        }
    }
}

/// Implements the CMSIS DAP descriptors.
pub struct ProbeUsb {
    device: UsbDevice<'static, UsbBus>,
    device_state: UsbDeviceState,
    winusb: MicrosoftDescriptors,
    dap_v1: CmsisDapV1<'static, UsbBus>,
    dap_v2: CmsisDapV2<'static, UsbBus>,
    serial: SerialPort<'static, UsbBus>,
    debug: DebugInterface<'static, UsbBus>,
}

const MANUFACTURER: &'static str = "Probe-rs development team";
const PRODUCT: &'static str = "Rusty Probe with CMSIS-DAP v1/v2 Support";

impl ProbeUsb {
    #[inline(always)]
    pub fn new(
        usb_bus: &'static UsbBusAllocator<UsbBus>,
        #[cfg(feature = "defmt-bbq")] defmt_consumer: defmt_brtt::DefmtConsumer,
    ) -> Self {
        let winusb = MicrosoftDescriptors;

        let dap_v1 = CmsisDapV1::new(64, usb_bus);
        let dap_v2 = CmsisDapV2::new(64, usb_bus);
        let serial = SerialPort::new(&usb_bus);
        let debug = DebugInterface::new(
            &usb_bus,
            64,
            #[cfg(feature = "defmt-bbq")]
            defmt_consumer,
        );

        let id = crate::device_signature::device_id_hex();
        info!("Device ID: {}", id);

        let descriptors_en = StringDescriptors::new(LangID::EN)
            .manufacturer(MANUFACTURER)
            .product(PRODUCT)
            .serial_number(id);

        let descriptors_en_us = StringDescriptors::new(LangID::EN_US)
            .manufacturer(MANUFACTURER)
            .product(PRODUCT)
            .serial_number(id);

        let device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x4853))
            .strings(&[descriptors_en, descriptors_en_us])
            .unwrap() // unwrap: Error is returned only if more than 16 languages are supplied.
            .device_class(0)
            .max_packet_size_0(64)
            .unwrap() // unwrap: 64 is a valid packet size
            .max_power(500)
            .unwrap() // unwrap: 500 is a valid power value
            .build();

        let device_state = device.state();

        ProbeUsb {
            device,
            device_state,
            winusb,
            dap_v1,
            dap_v2,
            serial,
            debug,
        }
    }

    pub fn flush_logs(&mut self) {
        #[cfg(feature = "defmt-bbq")]
        {
            if self.device.state() == UsbDeviceState::Configured {
                self.debug.pump_consumer();
            }
        }
    }

    pub fn interrupt(&mut self) -> Option<Request> {
        if self.device.poll(&mut [
            &mut self.winusb,
            &mut self.dap_v1,
            &mut self.dap_v2,
            &mut self.serial,
            &mut self.debug,
        ]) {
            let old_state = self.device_state;
            let new_state = self.device.state();
            self.device_state = new_state;

            if (old_state != new_state) && (new_state != UsbDeviceState::Configured) {
                return Some(Request::Suspend);
            }

            // Discard data from the serial interface
            let mut buf = [0; 64 as usize];
            let _read_data = self.serial.read(&mut buf);

            #[cfg(feature = "usb-serial-reboot")]
            match self.debug.read(&mut buf) {
                Ok(read_data) => {
                    if &buf[..read_data] == &0xDABAD000u32.to_be_bytes() {
                        rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
                    }
                }
                Err(UsbError::WouldBlock) => {}
                Err(e) => {
                    error!("Debug Interface ep read: {}", defmt::Debug2Format(&e));
                }
            }

            let r = self.dap_v1.process();
            if r.is_some() {
                return r;
            }

            let r = self.dap_v2.process();
            if r.is_some() {
                return r;
            }
        }
        None
    }

    /// Transmit a DAP report back over the DAPv1 HID interface
    pub fn dap1_reply(&mut self, data: &[u8]) {
        self.dap_v1
            .write_packet(data)
            .expect("DAPv1 EP write failed");
    }

    /// Transmit a DAP report back over the DAPv2 bulk interface
    pub fn dap2_reply(&mut self, data: &[u8]) {
        self.dap_v2
            .write_packet(data)
            .expect("DAPv2 EP write failed");
    }
}
