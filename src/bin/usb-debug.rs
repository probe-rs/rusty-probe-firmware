#![no_std]
#![no_main]

use rp_rtic as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use debug_traits::usb::{dap_v1::CmsisDapV1, dap_v2::CmsisDapV2};
    use defmt::*;
    use rp_pico::{
        hal::{clocks::init_clocks_and_plls, usb::UsbBus, watchdog::Watchdog},
        XOSC_CRYSTAL_FREQ,
    };
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_hid::descriptor::generator_prelude::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        probe_usb: rp_rtic::usb::ProbeUsb,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = defmt::unwrap!(init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok());

        let usb_bus: &'static _ = c.local.usb_bus.insert(UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));

        let probe_usb = rp_rtic::usb::ProbeUsb::new(&usb_bus);

        (Shared {}, Local { probe_usb }, init::Monotonics())
    }

    #[task(binds = USBCTRL_IRQ, shared = [], local = [probe_usb])]
    fn on_usb(ctx: on_usb::Context) {
        let probe_usb = ctx.local.probe_usb;

        if let Some(request) = probe_usb.interrupt() {
            info!("Got USB request: {}", request);
        }
    }
}
