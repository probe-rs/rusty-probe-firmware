use debug_traits::usb::{
    dap_v1::CmsisDapV1, dap_v2::CmsisDapV2, Request, DAP1_PACKET_SIZE, DAP2_PACKET_SIZE,
};
use defmt::*;
use rp_pico::hal::usb::UsbBus;
use usb_device::{class_prelude::*, prelude::*};
use usbd_hid::descriptor::generator_prelude::*;
use usbd_serial::SerialPort;

/// Implements the CMSIS DAP descriptors.
pub struct ProbeUsb {
    device: UsbDevice<'static, UsbBus>,
    device_state: UsbDeviceState,
    // winusb: MicrosoftDescriptors,
    dap_v1: CmsisDapV1<'static, UsbBus>,
    dap_v2: CmsisDapV2<'static, UsbBus>,
    serial: SerialPort<'static, UsbBus>,
    // dfu: DfuRuntime,
}

impl ProbeUsb {
    pub fn new(usb_bus: &'static UsbBusAllocator<UsbBus>) -> Self {
        let dap_v1 = CmsisDapV1::new(usb_bus);
        let dap_v2 = CmsisDapV2::new(usb_bus);
        let serial = SerialPort::new(&usb_bus);

        let id = crate::device_signature::device_id_hex();
        info!("Device ID: {}", id);
        let device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x4853))
            .manufacturer("Probe-rs development team")
            .product("Pico-Probe with CMSIS-DAP v1/v2 Support")
            .serial_number(id)
            .device_class(0)
            .max_packet_size_0(64)
            .max_power(500)
            .build();
        let device_state = device.state();

        ProbeUsb {
            device,
            device_state,
            dap_v1,
            dap_v2,
            serial,
        }
    }
    pub fn interrupt(&mut self) -> Option<Request> {
        if self.device.poll(&mut [
            // &mut usb.winusb,
            &mut self.dap_v1,
            &mut self.dap_v2,
            &mut self.serial,
            // &mut usb.dfu,
        ]) {
            let old_state = self.device_state;
            let new_state = self.device.state();
            self.device_state = new_state;
            if (old_state != new_state) && (new_state != UsbDeviceState::Configured) {
                return Some(Request::Suspend);
            }

            let r = self.dap_v1.process();
            if r.is_some() {
                return r;
            }

            let r = self.dap_v2.process();
            if r.is_some() {
                return r;
            }

            // Discard data from the serial interface
            let mut buf = [0; DAP2_PACKET_SIZE as usize];
            let _ = self.serial.read(&mut buf);
        }
        None
    }
}
