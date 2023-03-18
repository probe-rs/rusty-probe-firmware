#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use pico_probe as _;

#[rtic::app(device = rp2040_hal::pac, dispatchers = [XIP_IRQ, CLOCKS_IRQ])]
mod app {
    use core::mem::MaybeUninit;
    use embedded_hal::digital::v2::OutputPin;
    use pico_probe::setup::*;
    use rp2040_hal::gpio::PinState;
    use rp2040_hal::usb::UsbBus;
    use usb_device::class_prelude::*;

    use rtic_monotonics::systick::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        leds: BoardLeds,
        adc: AdcReader,
        trans_power: TranslatorPower,
        target_power: TargetPower,
        io: AllIOs,
    }

    #[init(local = [
        usb_bus: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit(),
        delay: MaybeUninit<pico_probe::systick_delay::Delay> = MaybeUninit::uninit(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let pac = cx.device;

        let mut resets = pac.RESETS;
        let mut watchdog = rp2040_hal::Watchdog::new(pac.WATCHDOG);
        let _clocks = if let Ok(clocks) = rp2040_hal::clocks::init_clocks_and_plls(
            12_000_000,
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

        let sio = rp2040_hal::Sio::new(pac.SIO);
        let pins =
            rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

        let mut led_red = pins.gpio27.into_push_pull_output();
        led_red.set_high().ok();
        let mut led_green = pins.gpio28.into_push_pull_output();
        led_green.set_high().ok();
        let mut led_blue = pins.gpio29.into_push_pull_output();
        led_blue.set_high().ok();

        // Enable ADC
        let adc = rp2040_hal::Adc::new(pac.ADC, &mut resets);

        // Enable the temperature sense channel
        // let mut temperature_sensor = adc.enable_temp_sensor();

        // Configure GPIO26 as an ADC input
        let adc_pin_0 = pins.gpio26.into_floating_input();

        let adc = AdcReader {
            pin: adc_pin_0,
            adc,
        };

        let pwm_slices = rp2040_hal::pwm::Slices::new(pac.PWM, &mut resets);

        ///////////////////////////////////
        // Voltage translator power
        ///////////////////////////////////

        let pwm = pwm_slices.pwm2;

        let mut translator_power = TranslatorPower::new(pins.gpio5.into_push_pull_output(), pwm);

        translator_power.set_vtranslator(3300);

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

        target_power.enable_vtgt();
        target_power.set_vtgt(3300);
        target_power.enable_5v_key();

        cortex_m::asm::delay(1_000_000);

        // All directions to outputs
        let _dir_io = pins.gpio12.into_push_pull_output_in_state(PinState::High);
        let _dir_ck = pins.gpio19.into_push_pull_output_in_state(PinState::High);
        let _dir_tdi = pins.gpio23.into_push_pull_output_in_state(PinState::High);
        let _dir_tdo_swo = pins.gpio22.into_push_pull_output_in_state(PinState::High);
        let _dir_vcp_rx = pins.gpio25.into_push_pull_output_in_state(PinState::High);
        let _dir_vcp_tx = pins.gpio24.into_push_pull_output_in_state(PinState::High);

        // IOs to struct
        let io = pins.gpio10.into_push_pull_output_in_state(PinState::Low);
        let ck = pins.gpio11.into_push_pull_output_in_state(PinState::Low);
        let tdi = pins.gpio17.into_push_pull_output_in_state(PinState::Low);
        let tdo_swo = pins.gpio16.into_push_pull_output_in_state(PinState::Low);
        let reset = pins.gpio9.into_push_pull_output_in_state(PinState::Low);
        let vcp_rx = pins.gpio21.into_push_pull_output_in_state(PinState::Low);
        let vcp_tx = pins.gpio20.into_push_pull_output_in_state(PinState::Low);

        let all_io = AllIOs {
            io,
            ck,
            tdi,
            tdo_swo,
            reset,
            vcp_rx,
            vcp_tx,
        };

        let leds = BoardLeds {
            red: led_red,
            green: led_green,
            blue: led_blue,
        };

        let systick_token = rtic_monotonics::make_systick_handler!();
        Systick::start(cx.core.SYST, 125_000_000, systick_token);

        led_test::spawn().ok();
        adc_test::spawn().ok();
        io_test::spawn().ok();

        (
            Shared {},
            Local {
                leds,
                adc,
                trans_power: translator_power,
                target_power,
                io: all_io,
            },
        )
    }

    #[task(local = [leds])]
    async fn led_test(cx: led_test::Context) {
        loop {
            cx.local.leds.red(true);
            Systick::delay(100.millis()).await;
            cx.local.leds.red(false);
            Systick::delay(100.millis()).await;

            cx.local.leds.green(true);
            Systick::delay(100.millis()).await;
            cx.local.leds.green(false);
            Systick::delay(100.millis()).await;

            cx.local.leds.blue(true);
            Systick::delay(100.millis()).await;
            cx.local.leds.blue(false);
            Systick::delay(100.millis()).await;
        }
    }

    #[task(local = [adc, target_power, trans_power])]
    async fn adc_test(cx: adc_test::Context) {
        loop {
            let val = cx.local.adc.voltage();
            defmt::println!("Vtgt = {} mV", val);
            Systick::delay(1000.millis()).await;
        }
    }

    #[task(local = [io])]
    async fn io_test(cx: io_test::Context) {
        let io = cx.local.io;
        loop {
            io.io.set_high().ok();
            Systick::delay(10.millis()).await;
            io.io.set_low().ok();

            io.ck.set_high().ok();
            Systick::delay(10.millis()).await;
            io.ck.set_low().ok();

            io.reset.set_high().ok();
            Systick::delay(10.millis()).await;
            io.reset.set_low().ok();

            io.tdi.set_high().ok();
            Systick::delay(10.millis()).await;
            io.tdi.set_low().ok();

            io.tdo_swo.set_high().ok();
            Systick::delay(10.millis()).await;
            io.tdo_swo.set_low().ok();

            io.vcp_rx.set_high().ok();
            Systick::delay(10.millis()).await;
            io.vcp_rx.set_low().ok();

            io.vcp_tx.set_high().ok();
            Systick::delay(10.millis()).await;
            io.vcp_tx.set_low().ok();

            Systick::delay(100.millis()).await;
        }
    }
}
