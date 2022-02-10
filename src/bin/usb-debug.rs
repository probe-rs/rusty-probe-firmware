#![no_std]
#![no_main]

use rp_rtic as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use super::*;
    use debug_traits::usb::{
        dap_v1::CmsisDapV1, dap_v2::CmsisDapV2, Request, DAP1_PACKET_SIZE, DAP2_PACKET_SIZE,
    };
    use defmt::*;
    use rp_pico::{
        hal::{clocks::init_clocks_and_plls, usb::UsbBus, watchdog::Watchdog},
        XOSC_CRYSTAL_FREQ,
    };
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_hid::descriptor::generator_prelude::*;

    pub struct DebugUsb {
        device: UsbDevice<'static, UsbBus>,
        device_state: UsbDeviceState,
        // winusb: MicrosoftDescriptors,
        dap_v1: CmsisDapV1<'static, UsbBus>,
        dap_v2: CmsisDapV2<'static, UsbBus>,
        // serial: SerialPort<'static, UsbBusType>,
        // dfu: DfuRuntime,
    }

    impl DebugUsb {
        pub fn interrupt(&mut self) -> Option<Request> {
            if self.device.poll(&mut [
                // &mut usb.winusb,
                &mut self.dap_v1,
                &mut self.dap_v2,
                // &mut usb.serial,
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
                // let mut buf = [0; DAP2_PACKET_SIZE as usize];
                // let _ = self.serial.read(&mut buf);
            }
            None
        }
    }

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        debug_usb: DebugUsb,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let usb_bus: &'static _ = c.local.usb_bus.insert(UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));

        let dap_v1 = CmsisDapV1::new(&usb_bus);
        let dap_v2 = CmsisDapV2::new(&usb_bus);
        // let serial = SerialPort::new(&usb_bus);

        let device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x4853))
            .manufacturer("Probe-rs development team")
            .product("RS-Probe with CMSIS-DAP v1/v2 Support")
            .serial_number("asdf234asdf2552")
            .device_class(0)
            .max_packet_size_0(64)
            .max_power(500)
            .build();
        let device_state = device.state();

        let debug_usb = DebugUsb {
            device,
            device_state,
            dap_v1,
            dap_v2,
        };

        (Shared {}, Local { debug_usb }, init::Monotonics())
    }

    #[task(binds = USBCTRL_IRQ, shared = [], local = [debug_usb])]
    fn on_usb(ctx: on_usb::Context) {
        let debug_usb = ctx.local.debug_usb;

        if let Some(request) = debug_usb.interrupt() {
            info!("Got USB request: {}", request);
        }
        // ctx.shared.hid.lock(|hid| if !usb_dev.poll(&mut [hid]) {});
    }
}
