use crate::dap::{Context, Jtag, Leds, Swd, Swo, Wait};
use crate::systick_delay::Delay;
use crate::{dap, usb::ProbeUsb};
use core::mem::MaybeUninit;
use rp2040_monotonic::Rp2040Monotonic;
use rp_pico::{
    hal::{
        clocks::init_clocks_and_plls,
        gpio::{pin::bank0::*, OutputDriveStrength, OutputSlewRate, Pin, Pins, PushPullOutput},
        pac,
        usb::UsbBus,
        watchdog::Watchdog,
        Clock, Sio,
    },
    XOSC_CRYSTAL_FREQ,
};
use usb_device::class_prelude::UsbBusAllocator;

pub type DapHandler = dap_rs::dap::Dap<'static, Context, Leds, Wait, Jtag, Swd, Swo>;
pub type LedPin = Pin<Gpio25, PushPullOutput>;

#[inline(always)]
pub fn setup(
    pac: pac::Peripherals,
    core: cortex_m::Peripherals,
    usb_bus: &'static mut MaybeUninit<UsbBusAllocator<UsbBus>>,
    delay: &'static mut MaybeUninit<Delay>,
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

    let usb_bus: &'static _ = usb_bus.write(UsbBusAllocator::new(UsbBus::new(
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
    let mut io = pins.gpio14;
    let mut ck = pins.gpio15;
    let reset = pins.gpio13;

    // High speed IO
    io.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    io.set_slew_rate(OutputSlewRate::Fast);
    ck.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    ck.set_slew_rate(OutputSlewRate::Fast);

    let delay = delay.write(Delay::new(core.SYST, clocks.system_clock.freq().raw()));

    const GIT_VERSION: &'static str = git_version::git_version!();

    let dap_hander = dap::create_dap(
        GIT_VERSION,
        io.into(),
        ck.into(),
        reset.into(),
        clocks.system_clock.freq().raw(),
        delay,
    );

    let mono = Rp2040Monotonic::new(pac.TIMER);

    (mono, led, probe_usb, dap_hander)
}
