#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use pico_probe as _;

#[rtic::app(device = rp2040_hal::pac, dispatchers = [XIP_IRQ, CLOCKS_IRQ])]
mod app {
    use core::mem::MaybeUninit;
    use pico_probe::setup::*;
    use rp2040_hal::usb::UsbBus;
    use usb_device::class_prelude::*;

    use rtic_monotonics::rp2040::{ExtU64, Timer};
    rtic_monotonics::make_rp2040_monotonic_handler!();

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        probe_usb: pico_probe::usb::ProbeUsb,
        dap_handler: DapHandler,
        leds: BoardLeds,
        adc: AdcReader,
    }

    #[init(local = [
        usb_bus: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit(),
        delay: MaybeUninit<pico_probe::systick_delay::Delay> = MaybeUninit::uninit(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let (leds, probe_usb, dap_handler, adc, translator_power, target_power) =
            setup(cx.device, cx.core, cx.local.usb_bus, cx.local.delay);

        led_blinker::spawn().ok();

        (
            Shared {},
            Local {
                probe_usb,
                dap_handler,
                leds,
                adc,
            },
        )
    }

    #[task(local = [leds, adc])]
    async fn led_blinker(cx: led_blinker::Context) {
        loop {
            cx.local.leds.toggle_green();
            let val = cx.local.adc.voltage();
            defmt::info!("Vtgt = {} mV", val);

            Timer::delay(1000.millis()).await;
        }
    }

    #[task(binds = USBCTRL_IRQ, local = [probe_usb, dap_handler, resp_buf: [u8; 64] = [0; 64]])]
    fn on_usb(ctx: on_usb::Context) {
        let probe_usb = ctx.local.probe_usb;
        let dap = ctx.local.dap_handler;
        let resp_buf = ctx.local.resp_buf;

        if let Some(request) = probe_usb.interrupt() {
            use dap_rs::{dap::DapVersion, usb::Request};

            match request {
                Request::DAP1Command((report, n)) => {
                    let len = dap.process_command(&report[..n], resp_buf, DapVersion::V1);

                    if len > 0 {
                        probe_usb.dap1_reply(&resp_buf[..len]);
                    }
                }
                Request::DAP2Command((report, n)) => {
                    let len = dap.process_command(&report[..n], resp_buf, DapVersion::V2);

                    if len > 0 {
                        probe_usb.dap2_reply(&resp_buf[..len]);
                    }
                }
                Request::Suspend => {
                    defmt::info!("Got USB suspend command");
                    dap.suspend();
                }
            }
        }
    }
}
