#![no_std]
#![no_main]

use rp_rtic as _;

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [XIP_IRQ])]
mod app {
    use defmt::*;
    use embedded_hal::digital::v2::ToggleableOutputPin;
    use rp2040_monotonic::*;
    use rp_pico::hal::usb::UsbBus;
    use rp_rtic::setup::*;
    use usb_device::class_prelude::*;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Monotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        probe_usb: rp_rtic::usb::ProbeUsb,
        led: LedPin,
        swd: Swd,
    }

    #[init(local = [usb_bus: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let (mono, led, probe_usb, swd) = setup(cx.device, cx.local.usb_bus);

        led_blinker::spawn().ok();
        swd_sender::spawn().ok();

        (
            Shared {},
            Local {
                probe_usb,
                led,
                swd,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [led])]
    fn led_blinker(cx: led_blinker::Context) {
        cx.local.led.toggle().ok();
        led_blinker::spawn_after(500.millis()).ok();
    }

    #[task(local = [swd])]
    fn swd_sender(cx: swd_sender::Context) {
        info!("Sending SWD");
        cx.local.swd.write(0x0b, 0x12345678);
        swd_sender::spawn_after(100.millis()).ok();
    }

    #[task(binds = USBCTRL_IRQ, shared = [], local = [probe_usb])]
    fn on_usb(ctx: on_usb::Context) {
        let probe_usb = ctx.local.probe_usb;

        if let Some(request) = probe_usb.interrupt() {
            info!("Got USB request: {}", request);
        }
    }
}
