use crate::dap::{Context, Jtag, Leds, Swd, Swo, Wait};
use crate::systick_delay::Delay;
use crate::{dap, usb::ProbeUsb};
use core::mem::MaybeUninit;
use embedded_hal::adc::OneShot;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::PwmPin;
use rp2040_hal::{
    clocks::init_clocks_and_plls,
    gpio::{
        pin::bank0::*, FloatingInput, OutputDriveStrength, OutputSlewRate, Pin, Pins,
        PushPullOutput,
    },
    pac,
    pwm::{self, FreeRunning, Pwm0, Pwm2, Slice},
    usb::UsbBus,
    watchdog::Watchdog,
    Adc, Clock, Sio,
};
use rtic_monotonics::rp2040;
use usb_device::class_prelude::UsbBusAllocator;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

pub type DapHandler = dap_rs::dap::Dap<'static, Context, Leds, Wait, Jtag, Swd, Swo>;

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
    BoardLeds,
    ProbeUsb,
    DapHandler,
    TargetVccReader,
    TranslatorPower,
    TargetPower,
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
    let adc_pin_0 = pins.gpio26.into_floating_input();

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

    let dap_hander = dap::create_dap(
        git_version,
        io.into(),
        ck.into(),
        reset.into(),
        dir_io.into(),
        dir_ck.into(),
        clocks.system_clock.freq().raw(),
        delay,
    );

    let timer_token = rtic_monotonics::create_rp2040_monotonic_token!();
    rp2040::Timer::start(pac.TIMER, &mut resets, timer_token);

    (
        BoardLeds {
            red: led_red,
            green: led_green,
            blue: led_blue,
        },
        probe_usb,
        dap_hander,
        adc,
        translator_power,
        target_power,
    )
}

pub struct AllIOs {
    pub io: Pin<Gpio10, PushPullOutput>,
    pub ck: Pin<Gpio11, PushPullOutput>,
    pub tdi: Pin<Gpio17, PushPullOutput>,
    pub tdo_swo: Pin<Gpio16, PushPullOutput>,
    pub reset: Pin<Gpio9, PushPullOutput>,
    pub vcp_rx: Pin<Gpio21, PushPullOutput>,
    pub vcp_tx: Pin<Gpio20, PushPullOutput>,
}

pub struct BoardLeds {
    pub green: Pin<Gpio27, PushPullOutput>,
    pub red: Pin<Gpio28, PushPullOutput>,
    pub blue: Pin<Gpio29, PushPullOutput>,
}

impl BoardLeds {
    pub fn red(&mut self, level: bool) {
        self.red.set_state((!level).into()).ok();
    }

    pub fn toggle_red(&mut self) {
        self.red.toggle().ok();
    }

    pub fn green(&mut self, level: bool) {
        self.green.set_state((!level).into()).ok();
    }

    pub fn toggle_green(&mut self) {
        self.green.toggle().ok();
    }

    pub fn blue(&mut self, level: bool) {
        self.blue.set_state((!level).into()).ok();
    }
    pub fn toggle_blue(&mut self) {
        self.blue.toggle().ok();
    }

    pub fn rgb(&mut self, r: bool, g: bool, b: bool) {
        self.red(r);
        self.green(g);
        self.blue(b);
    }
}

pub struct TargetVccReader {
    pub pin: Pin<Gpio26, FloatingInput>,
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
        mut vtranslator_pin: Pin<Gpio5, PushPullOutput>,
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
        channel.set_duty(1023);

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
        channel.set_duty(cnt as u16);
    }
}

pub struct TargetPower {
    enable_5v_key: Pin<Gpio3, PushPullOutput>,
    enable_vtgt: Pin<Gpio6, PushPullOutput>,
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
        mut enable_5v_key: Pin<Gpio3, PushPullOutput>,
        mut enable_5v: Pin<Gpio7, PushPullOutput>,
        mut enable_vtgt: Pin<Gpio6, PushPullOutput>,
        mut vtgt_pin: Pin<Gpio0, PushPullOutput>,
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
        channel.set_duty(1023);

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
        channel.set_duty(cnt as u16);
    }
}
