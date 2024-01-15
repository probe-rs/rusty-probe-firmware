use crate::dap::{Context, Jtag, Swd, Swo, Wait};
use crate::leds::{BoardLeds, HostStatusToken, LedManager};
use crate::systick_delay::Delay;
use crate::{dap, usb::ProbeUsb};
use core::mem::MaybeUninit;
use dap_rs::usb_device::class_prelude::UsbBusAllocator;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal_02::adc::OneShot;
use replace_with::replace_with_or_abort_unchecked;
use rp2040_hal::adc::AdcPin;
use rp2040_hal::gpio::bank0::{
    Gpio0, Gpio10, Gpio11, Gpio16, Gpio17, Gpio20, Gpio21, Gpio26, Gpio3, Gpio5, Gpio6, Gpio7,
    Gpio8, Gpio9,
};
use rp2040_hal::gpio::{
    FunctionSio, FunctionSioInput, FunctionSioOutput, PinId, PinState, PullDown, PullNone,
    PullType, PullUp, SioInput, SioOutput, ValidFunction,
};
use rp2040_hal::{
    clocks::init_clocks_and_plls,
    gpio::{OutputDriveStrength, OutputSlewRate, Pin, Pins},
    pac,
    pwm::{self, FreeRunning, Pwm0, Pwm2, Slice},
    rom_data,
    usb::UsbBus,
    watchdog::Watchdog,
    Adc, Clock, Sio,
};
use rtic_monotonics::rp2040::prelude::*;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

pub type DapHandler = dap_rs::dap::Dap<'static, Context, HostStatusToken, Wait, Jtag, Swd, Swo>;
rp2040_timer_monotonic!(Mono);

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[inline(always)]
pub fn setup(
    pac: pac::Peripherals,
    core: cortex_m::Peripherals,
    usb_bus: &'static mut MaybeUninit<UsbBusAllocator<UsbBus>>,
    delay: &'static mut MaybeUninit<Delay>,
) -> (
    LedManager,
    ProbeUsb,
    DapHandler,
    TargetVccReader,
    TranslatorPower,
    TargetPower,
    TargetPhysicallyConnected,
) {
    let mut resets = pac.RESETS;
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = if let Ok(clocks) = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    ) {
        clocks
    } else {
        unreachable!()
    };

    let usb_bus: &'static _ = usb_bus.write(UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut resets,
    )));

    #[cfg(feature = "defmt-bbq")]
    let consumer = defmt_brtt::init().ok().unwrap();

    let rom_version = rom_data::rom_version_number();
    let git_revision = rom_data::git_revision();
    let copyright = rom_data::copyright_string();

    defmt::info!(
        "RP2040-B{=u8} (ROM {=u32:x}) {=str}",
        rom_version,
        git_revision,
        copyright
    );

    let probe_usb = ProbeUsb::new(
        &usb_bus,
        #[cfg(feature = "defmt-bbq")]
        consumer,
    );

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

    let mut led_green = pins.gpio27.into_push_pull_output();
    led_green.set_high().ok();
    let mut led_red = pins.gpio28.into_push_pull_output();
    led_red.set_high().ok();
    let mut led_blue = pins.gpio29.into_push_pull_output();
    led_blue.set_high().ok();

    // Enable ADC
    let adc = Adc::new(pac.ADC, &mut resets);

    // Enable the temperature sense channel
    // let mut temperature_sensor = adc.enable_temp_sensor();

    // Configure GPIO26 as an ADC input
    let adc_pin_0 = AdcPin::new(pins.gpio26.into_floating_input()).expect("Not an ADC pin");

    let adc = TargetVccReader {
        pin: adc_pin_0,
        adc,
    };

    let pwm_slices = pwm::Slices::new(pac.PWM, &mut resets);

    ///////////////////////////////////
    // Voltage translator power
    ///////////////////////////////////

    let pwm = pwm_slices.pwm2;

    let mut translator_power = TranslatorPower::new(pins.gpio5.into_push_pull_output(), pwm);

    translator_power.set_translator_vcc(1800);

    ///////////////////////////////////
    // Target power
    ///////////////////////////////////

    // Configure PWM
    let pwm = pwm_slices.pwm0;

    let mut target_power = TargetPower::new(
        pins.gpio3.into_push_pull_output(),
        pins.gpio7.into_push_pull_output(),
        pins.gpio6.into_push_pull_output(),
        pins.gpio0.into_push_pull_output(),
        pwm,
    );

    // target_power.enable_vtgt();
    target_power.set_vtgt(1800);

    cortex_m::asm::delay(1_000_000);

    let mut io = pins.gpio10;
    let mut ck = pins.gpio11;
    let mut dir_io = pins.gpio12;
    let mut dir_ck = pins.gpio19;
    let reset = pins.gpio9;
    let gnd_detect = pins.gpio8.into_pull_up_input();

    let _tdi = pins.gpio17;
    let _dir_tdi = pins.gpio23;

    let _vcp_rx = pins.gpio21;
    let _vcp_tx = pins.gpio20;
    let _dir_vcp_rx = pins.gpio25;
    let _dir_vcp_tx = pins.gpio24;

    // High speed IO
    io.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    io.set_slew_rate(OutputSlewRate::Fast);
    ck.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    ck.set_slew_rate(OutputSlewRate::Fast);
    dir_io.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    dir_io.set_slew_rate(OutputSlewRate::Fast);
    dir_ck.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
    dir_ck.set_slew_rate(OutputSlewRate::Fast);

    let git_version: &'static str = git_version::git_version!();
    let delay = delay.write(Delay::new(core.SYST, clocks.system_clock.freq().raw()));

    let board_leds = BoardLeds::new(led_red, led_green, led_blue);
    let mut led_manager = LedManager::new(board_leds);
    let host_status_token = led_manager.host_status_token();
    let target_physically_connected = TargetPhysicallyConnected { pin: gnd_detect };

    let dap_hander = dap::create_dap(
        git_version,
        DynPin::Input(io.into_pull_down_input()),
        DynPin::Input(ck.into_pull_down_input()),
        DynPin::Input(reset.into_floating_input()),
        dir_io.into_push_pull_output().into_pull_type(),
        dir_ck.into_push_pull_output().into_pull_type(),
        clocks.system_clock.freq().raw(),
        delay,
        host_status_token,
    );

    Mono::start(pac.TIMER, &mut resets);

    (
        led_manager,
        probe_usb,
        dap_hander,
        adc,
        translator_power,
        target_power,
        target_physically_connected,
    )
}

/// A simple Dynamic pin, meaning it can be output or input.
pub enum DynPin<P, InputPull>
where
    P: PinId,
    InputPull: PullType,
{
    /// The pin is in input mode.
    Input(Pin<P, FunctionSioInput, InputPull>),
    /// The pin is in output mode.
    Output(Pin<P, FunctionSioOutput, InputPull>),
}

impl<P, InputPull> DynPin<P, InputPull>
where
    P: ValidFunction<FunctionSioInput>,
    P: ValidFunction<FunctionSioOutput>,
    InputPull: PullType,
{
    pub fn into_input(&mut self) {
        unsafe {
            replace_with_or_abort_unchecked(self, |p| match p {
                DynPin::Input(_) => p,
                DynPin::Output(o) => DynPin::Input(o.into_function().into_pull_type()),
            })
        }
    }

    pub fn into_output(&mut self) {
        unsafe {
            replace_with_or_abort_unchecked(self, |p| match p {
                DynPin::Input(i) => DynPin::Output(i.into_push_pull_output()),
                DynPin::Output(_) => p,
            })
        }
    }

    pub fn into_output_in_state(&mut self, state: PinState) {
        unsafe {
            replace_with_or_abort_unchecked(self, |p| {
                match p {
                    DynPin::Input(i) => {
                        //
                        DynPin::Output(i.into_push_pull_output_in_state(state))
                    }
                    DynPin::Output(_) => p,
                }
            })
        }
    }

    /// Note: This function panics if the pin is in the wrong mode.
    pub fn set_high(&mut self) {
        match self {
            DynPin::Input(_) => defmt::panic!("Output operation on input pin"),
            DynPin::Output(o) => {
                o.set_high().ok();
            }
        }
    }

    /// Note: This function panics if the pin is in the wrong mode.
    pub fn set_low(&mut self) {
        match self {
            DynPin::Input(_) => defmt::panic!("Output operation on input pin"),
            DynPin::Output(o) => {
                o.set_high().ok();
            }
        }
    }

    /// Note: This function panics if the pin is in the wrong mode.
    pub fn is_high(&mut self) -> bool {
        match self {
            DynPin::Input(i) => matches!(i.is_high(), Ok(true)),
            DynPin::Output(_) => {
                defmt::panic!("Input operation on output pin");
            }
        }
    }
}

pub struct AllIOs {
    pub io: DynPin<Gpio10, PullDown>,
    pub ck: DynPin<Gpio11, PullDown>,
    pub tdi: DynPin<Gpio17, PullDown>,
    pub tdo_swo: DynPin<Gpio16, PullDown>,
    pub reset: DynPin<Gpio9, PullNone>,
    pub vcp_rx: DynPin<Gpio21, PullUp>,
    pub vcp_tx: DynPin<Gpio20, PullUp>,
}

pub struct TargetPhysicallyConnected {
    pin: Pin<Gpio8, FunctionSioInput, PullUp>,
}

impl TargetPhysicallyConnected {
    /// This checks for the target being connected via the GND detect pin.
    pub fn target_detected(&mut self) -> bool {
        #[cfg(feature = "gnddetect")]
        return matches!(self.pin.is_low(), Ok(true));
        #[cfg(not(feature = "gnddetect"))]
        return true;
    }
}

pub struct TargetVccReader {
    pub pin: AdcPin<Pin<Gpio26, FunctionSio<SioInput>, PullNone>>,
    pub adc: Adc,
}

impl TargetVccReader {
    pub fn read_voltage_mv(&mut self) -> u32 {
        let Self { pin, adc } = self;
        let val: u16 = adc.read(pin).unwrap();

        (2 * 3300 * val as u32) / 4095
    }
}

pub struct TranslatorPower {
    vtranslator_pwm: Slice<Pwm2, FreeRunning>,
}

impl TranslatorPower {
    pub fn new(
        mut vtranslator_pin: Pin<Gpio5, FunctionSio<SioOutput>, PullDown>,
        mut vtranslator_pwm: Slice<Pwm2, FreeRunning>,
    ) -> Self {
        vtranslator_pwm.clr_ph_correct();
        vtranslator_pwm.set_top(4095);
        vtranslator_pwm.enable();

        vtranslator_pin.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
        vtranslator_pin.set_slew_rate(OutputSlewRate::Fast);

        // Output channel B on PWM2 to GPIO 5
        let channel = &mut vtranslator_pwm.channel_b;
        channel.output_to(vtranslator_pin);
        channel.set_duty_cycle(1023).unwrap();

        Self { vtranslator_pwm }
    }

    pub fn set_translator_vcc(&mut self, mv: u32) {
        // ans = 0.0900 * 4095
        let v33 = 369; // duty cycle that give 3.3v

        // ans = 0.7617 * 4095
        let v18 = 3119; // duty cycle that gives 1.8v

        let vomin = 1800; // The actual voltage when PWM is set to v18 duty cycle
        let vomax = 3300; // The actual voltage when PWM is set to v33 duty cycle

        let mv = mv.min(vomax).max(vomin);

        let limit_diff = v18 - v33;
        let mv_diff = mv - vomin;

        let cnt = ((vomax - vomin - mv_diff) * limit_diff) / (vomax - vomin) + v33;

        let channel = &mut self.vtranslator_pwm.channel_b;
        channel.set_duty_cycle(cnt as u16).unwrap();
    }
}

pub struct TargetPower {
    enable_5v_key: Pin<Gpio3, FunctionSioOutput, PullDown>,
    enable_vtgt: Pin<Gpio6, FunctionSioOutput, PullDown>,
    vtgt_pwm: Slice<Pwm0, FreeRunning>,
}

impl TargetPower {
    pub fn enable_vtgt(&mut self) {
        self.enable_vtgt.set_high().ok();
    }

    pub fn enable_5v_key(&mut self) {
        self.enable_5v_key.set_high().ok();
    }

    pub fn new(
        mut enable_5v_key: Pin<Gpio3, FunctionSioOutput, PullDown>,
        mut enable_5v: Pin<Gpio7, FunctionSioOutput, PullDown>,
        mut enable_vtgt: Pin<Gpio6, FunctionSioOutput, PullDown>,
        mut vtgt_pin: Pin<Gpio0, FunctionSioOutput, PullDown>,
        mut vtgt_pwm: Slice<Pwm0, FreeRunning>,
    ) -> Self {
        // Always enable the protected 5v (existed until rev E of hardware)
        enable_5v.set_high().ok();

        enable_5v_key.set_low().ok();
        enable_vtgt.set_low().ok();

        vtgt_pwm.clr_ph_correct();
        vtgt_pwm.set_top(4095);
        vtgt_pwm.enable();

        vtgt_pin.set_drive_strength(OutputDriveStrength::TwelveMilliAmps);
        vtgt_pin.set_slew_rate(OutputSlewRate::Fast);

        // Output channel A on PWM0 to GPIO 0
        let channel = &mut vtgt_pwm.channel_a;
        channel.output_to(vtgt_pin);
        channel.set_duty_cycle(1023).ok();

        Self {
            enable_5v_key,
            enable_vtgt,
            vtgt_pwm,
        }
    }

    pub fn set_vtgt(&mut self, mv: u32) {
        let v33 = 369; // duty cycle that give 3.3v
        let v18 = 3119; // duty cycle that gives 1.8v

        let vomin = 1800; // The actual voltage when PWM is set to v18 duty cycle
        let vomax = 3300; // The actual voltage when PWM is set to v33 duty cycle

        let mv = mv.min(vomax).max(vomin);

        let limit_diff = v18 - v33;
        let mv_diff = mv - vomin;

        let cnt = ((vomax - vomin - mv_diff) * limit_diff) / (vomax - vomin) + v33;

        let channel = &mut self.vtgt_pwm.channel_a;
        channel.set_duty_cycle(cnt as u16).unwrap();
    }
}
