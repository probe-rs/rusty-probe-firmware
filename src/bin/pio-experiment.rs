#![no_std]
#![no_main]

use pico_probe as _;

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [XIP_IRQ])]
mod app {
    use defmt::*;
    use rp2040_monotonic::*;
    use rp_pico::{
        hal::{clocks::init_clocks_and_plls, gpio::Pins, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Monotonic = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = cx.device.RESETS;
        let mut watchdog = Watchdog::new(cx.device.WATCHDOG);
        let _clocks = defmt::unwrap!(init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok());

        let mono = Rp2040Monotonic::new(cx.device.TIMER);

        let sio = Sio::new(cx.device.SIO);
        let pins = Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        //  let led = pins.gpio25.into_push_pull_output();
        let io = pins.gpio14;
        let ck = pins.gpio15;

        println!("Starting PIO");
        pico_probe::pio::setup_pio(&mut resets, cx.device.PIO0, ck, io);
        println!("Started PIO");

        (Shared {}, Local {}, init::Monotonics(mono))
    }
}
