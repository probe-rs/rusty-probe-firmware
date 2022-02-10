#![no_std]
#![no_main]

use rp_rtic as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ])]
mod app {
    use defmt::*;
    use embedded_hal::digital::v2::ToggleableOutputPin;
    use rp2040_monotonic::*;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Monotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: rp_rtic::setup::LedPin,
        swd: Option<rp_rtic::setup::Swd>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let (mono, led, swd) = rp_rtic::setup::setup(c.device);
        tick::spawn().ok();
        (
            Shared {},
            Local {
                led,
                swd: Some(swd),
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [led, swd])]
    fn tick(cx: tick::Context) {
        info!("Tick");
        tick::spawn_after(1.secs()).ok();

        cx.local.led.toggle().ok();

        let swd = cx.local.swd.take().unwrap();
        let swd = swd.write(0x51, 0xabcd1234);
        *cx.local.swd = Some(swd);
    }
}
