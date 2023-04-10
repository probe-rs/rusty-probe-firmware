#![no_std]
#![no_main]

use pico_probe as _;

#[rtic::app(device = rp2040_hal::pac, dispatchers = [XIP_IRQ])]
mod app {
    use defmt::*;
    use rp2040_hal::{clocks::init_clocks_and_plls, gpio::Pins, watchdog::Watchdog, Sio};
    use rtic_monotonics::systick::*;

    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
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

        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, 125_000_000, systick_token);

        let sio = Sio::new(cx.device.SIO);
        let pins = Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        //  let led = pins.gpio25.into_push_pull_output();
        let io = pins.gpio10;
        let io_dir = pins.gpio12;
        let ck = pins.gpio11;

        println!("Starting PIO");
        pico_probe::pio::setup_pio(&mut resets, cx.device.PIO0, ck, io, io_dir);
        println!("Started PIO");

        (Shared {}, Local {})
    }
}
