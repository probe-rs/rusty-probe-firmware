use dap_rs::usb::{dap_v1::CmsisDapV1, dap_v2::CmsisDapV2, winusb::MicrosoftDescriptors, Request};
use defmt::*;
use rp2040_hal::usb::UsbBus;
use usb_device::{class_prelude::*, prelude::*};
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

        let device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x4853))
            .manufacturer("Probe-rs development team")
            .product("Rusty Probe with CMSIS-DAP v1/v2 Support")
            .serial_number(id)
            .device_class(0)
            .max_packet_size_0(64)
            .max_power(500)
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

    pub fn serial_write(&mut self, data: &[u8]) -> usize {
        if self.device.state() == UsbDeviceState::Configured && self.serial.dtr() {
            self.serial.write(data).unwrap_or(0)
        } else {
            0
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

            // self.flush_serial_buffers();

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

    // pub fn flush_serial_buffers(&mut self) {
    //     if self.device.state() == UsbDeviceState::Configured {
    //         if let Ok(grant) = self.serial_consumer.read() {
    //             if let Ok(write_size) = self.serial.write(&grant) {
    //                 grant.release(write_size);
    //             }
    //         }
    //     }

    //     if let Ok(mut grant) = self.serial_producer.grant_max_remaining(64) {
    //         if let Ok(read_size) = self.serial.read(&mut grant) {
    //             #[cfg(feature = "usb-serial-reboot")]
    //             if &grant[..read_data] == &0xDABAD000u32.to_be_bytes() {
    //                 rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
    //             }
    //             if self.device.state() == UsbDeviceState::Configured {
    //                 grant.commit(read_size);
    //             }
    //         }
    //     }
    // }

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
