use rp2040_monotonic::Rp2040Monotonic;
use rp_pico::{
    hal::{
        clocks::init_clocks_and_plls,
        gpio::{pin::bank0::*, Pin, Pins, PushPullOutput},
        pac,
        usb::UsbBus,
        watchdog::Watchdog,
        Clock, Sio,
    },
    XOSC_CRYSTAL_FREQ,
};

use usb_device::class_prelude::UsbBusAllocator;

use crate::dap::{Context, Jtag, Leds, Swd, Swo, Wait};
use crate::{dap, usb::ProbeUsb};

pub type DapHandler = dap_rs::dap::Dap<'static, Context, Leds, Wait, Jtag, Swd, Swo>;
pub type LedPin = Pin<Gpio25, PushPullOutput>;

#[inline(always)]
pub fn setup(
    pac: pac::Peripherals,
    usb_bus: &'static mut Option<UsbBusAllocator<UsbBus>>,
) -> (Rp2040Monotonic, LedPin, ProbeUsb, DapHandler) {
    let mut resets = pac.RESETS;
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = defmt::unwrap!(init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    )
    .ok());

    let usb_bus: &'static _ = usb_bus.insert(UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut resets,
    )));

    let probe_usb = ProbeUsb::new(&usb_bus);

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

    let led = pins.gpio25.into_push_pull_output();
    let io = pins.gpio14;
    let ck = pins.gpio15;
    let reset = pins.gpio13;

    let dap_hander = dap::create_dap(
        "1.2.3-sdfsesd",
        io.into(),
        ck.into(),
        reset.into(),
        clocks.system_clock.freq().0,
    );

    let mono = Rp2040Monotonic::new(pac.TIMER);

    (mono, led, probe_usb, dap_hander)
}
