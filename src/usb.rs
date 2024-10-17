use dap_rs::usb::{dap_v1::CmsisDapV1, dap_v2::CmsisDapV2, winusb::MicrosoftDescriptors, Request};
use dap_rs::usb_device::{class_prelude::*, prelude::*};
use defmt::*;
use rp2040_hal::usb::UsbBus;
use usbd_serial::SerialPort;

/// Implements the CMSIS DAP descriptors.
pub struct ProbeUsb {
    device: UsbDevice<'static, UsbBus>,
    device_state: UsbDeviceState,
    winusb: MicrosoftDescriptors,
    dap_v1: CmsisDapV1<'static, UsbBus>,
    dap_v2: CmsisDapV2<'static, UsbBus>,
    serial: SerialPort<'static, UsbBus>,

    #[cfg(feature = "defmt-bbq")]
    defmt_consumer: defmt_brtt::DefmtConsumer,
}

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

        let id = crate::device_signature::device_id_hex();
        info!("Device ID: {}", id);

        let descriptors = StringDescriptors::new(LangID::EN)
            .manufacturer("Probe-rs development team")
            .product("Rusty Probe with CMSIS-DAP v1/v2 Support")
            .serial_number(id);

        let device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x4853))
            .strings(&[descriptors])
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
            #[cfg(feature = "defmt-bbq")]
            defmt_consumer,
        }
    }

    pub fn flush_logs(&mut self) {
        #[cfg(feature = "defmt-bbq")]
        {
            if self.device.state() == UsbDeviceState::Configured {
                if let Ok(grant) = self.defmt_consumer.read() {
                    let bytes_written = if let Ok(bytes_written) = self.serial.write(&grant) {
                        bytes_written
                    } else {
                        0
                    };
                    grant.release(bytes_written);
                }
            }
        }
    }

    pub fn interrupt(&mut self) -> Option<Request> {
        if self.device.poll(&mut [
            &mut self.winusb,
            &mut self.dap_v1,
            &mut self.dap_v2,
            &mut self.serial,
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
            if let Ok(read_data) = _read_data {
                if &buf[..read_data] == &0xDABAD000u32.to_be_bytes() {
                    rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
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
