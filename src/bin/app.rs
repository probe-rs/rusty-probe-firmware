#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use pico_probe as _;

#[rtic::app(device = rp2040_hal::pac, dispatchers = [XIP_IRQ, CLOCKS_IRQ])]
mod app {
    use core::mem::MaybeUninit;
    use pico_probe::{
        leds::{LedManager, Vtarget},
        setup::*,
    };
    use rp2040_hal::usb::UsbBus;
    use usb_device::class_prelude::*;

    use rtic_monotonics::rp2040::{ExtU64, Timer};

    #[shared]
    struct Shared {
        probe_usb: pico_probe::usb::ProbeUsb,
    }

    #[local]
    struct Local {
        dap_handler: DapHandler,
        target_vcc: TargetVccReader,
        translator_power: TranslatorPower,
        target_power: TargetPower,
        target_physically_connected: TargetPhysicallyConnected,
    }

    #[init(local = [
        usb_bus: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit(),
        delay: MaybeUninit<pico_probe::systick_delay::Delay> = MaybeUninit::uninit(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let (
            leds,
            probe_usb,
            dap_handler,
            target_vcc,
            translator_power,
            target_power,
            target_physically_connected,
        ) = setup(cx.device, cx.core, cx.local.usb_bus, cx.local.delay);

        voltage_translator_control::spawn().ok();

        #[cfg(feature = "defmt-bbq")]
        log_pump::spawn().ok();

        led_driver::spawn(leds).ok();

        (
            Shared { probe_usb },
            Local {
                dap_handler,
                target_vcc,
                translator_power,
                target_power,
                target_physically_connected,
            },
        )
    }

    #[task]
    async fn led_driver(_: led_driver::Context, mut leds: LedManager) {
        leds.run().await;
    }

    #[task(priority = 3, local = [target_vcc, translator_power, target_power, target_physically_connected])]
    async fn voltage_translator_control(cx: voltage_translator_control::Context) {
        loop {
            // Set the voltage translators to track Target's VCC
            let target_vcc_mv = cx.local.target_vcc.read_voltage_mv();

            if target_vcc_mv > 1500 {
                cx.local.translator_power.set_translator_vcc(target_vcc_mv);
            } else {
                if cx.local.target_physically_connected.target_detected() {
                    // If there is no VCC detected use 3.3v.
                    cx.local.translator_power.set_translator_vcc(3300);
                } else {
                    cx.local.translator_power.set_translator_vcc(0);
                }
            }

            defmt::trace!("Tracking Target VCC at {} mV", target_vcc_mv);

            if target_vcc_mv > 2500 {
                LedManager::set_current_vtarget(Some(Vtarget::Voltage3V3));
            } else if target_vcc_mv > 1500 {
                LedManager::set_current_vtarget(Some(Vtarget::Voltage1V8));
            } else {
                if cx.local.target_physically_connected.target_detected() {
                    // If there is no VCC detected use 3.3v.
                    LedManager::set_current_vtarget(Some(Vtarget::Voltage3V3));
                } else {
                    LedManager::set_current_vtarget(None);
                }
            };

            Timer::delay(100.millis()).await;
        }
    }

    #[task(shared = [probe_usb])]
    async fn log_pump(mut ctx: log_pump::Context) {
        loop {
            ctx.shared
                .probe_usb
                .lock(|probe_usb| probe_usb.flush_logs());
            Timer::delay(100.millis()).await;
        }
    }

    #[task(binds = USBCTRL_IRQ, priority = 2, shared = [ probe_usb ], local = [dap_handler, resp_buf: [u8; 64] = [0; 64]])]
    fn on_usb(ctx: on_usb::Context) {
        let mut probe_usb = ctx.shared.probe_usb;
        let dap = ctx.local.dap_handler;
        let resp_buf = ctx.local.resp_buf;

        probe_usb.lock(|probe_usb| {
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
        });
    }
}
