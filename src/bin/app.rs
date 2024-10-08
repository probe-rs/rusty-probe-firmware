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

    use rtic_monotonics::rp2040::{fugit::RateExtU32, ExtU64, Timer};

    struct UsbUartState {
        data_rate: u32,
        data_bits: u8,
        stop_bits: usbd_serial::StopBits,
        parity_type: usbd_serial::ParityType,
    }

    impl PartialEq<usbd_serial::LineCoding> for UsbUartState {
        fn eq(&self, other: &usbd_serial::LineCoding) -> bool {
            self.data_bits == other.data_bits() &&
                self.data_rate == other.data_rate() &&
                self.parity_type == other.parity_type() &&
                self.stop_bits == other.stop_bits()
        }
    }

    impl UsbUartState {
        pub fn new(line_coding: &usbd_serial::LineCoding) -> Self {
            Self {
                data_rate: line_coding.data_rate(),
                data_bits: line_coding.data_bits(),
                stop_bits: line_coding.stop_bits(),
                parity_type: line_coding.parity_type(),
            }
        }

        pub fn try_get_uart_config(&self) -> Option<rp2040_hal::uart::UartConfig> {
            let data_bits = match self.data_bits {
                5 => Some(rp2040_hal::uart::DataBits::Five),
                6 => Some(rp2040_hal::uart::DataBits::Six),
                7 => Some(rp2040_hal::uart::DataBits::Seven),
                8 => Some(rp2040_hal::uart::DataBits::Eight),
                _ => None,
            }?;
            let parity = match self.parity_type {
                // "Event" is probably a typo
                usbd_serial::ParityType::Event => Some(Some(rp2040_hal::uart::Parity::Even)),
                usbd_serial::ParityType::Odd => Some(Some(rp2040_hal::uart::Parity::Odd)),
                usbd_serial::ParityType::None => Some(None),
                _ => None,
            }?;
            let stop_bits = match self.stop_bits {
                usbd_serial::StopBits::One => Some(rp2040_hal::uart::StopBits::One),
                usbd_serial::StopBits::Two => Some(rp2040_hal::uart::StopBits::Two),
                _ => None
            }?;
            
            Some(rp2040_hal::uart::UartConfig::new(
                self.data_rate.Hz(),
                data_bits, parity, stop_bits))
        }
    }

    #[shared]
    struct Shared {
        probe_usb: pico_probe::usb::ProbeUsb,
        uart: Uart,
    }

    #[local]
    struct Local {
        dap_handler: DapHandler,
        target_vcc: TargetVccReader,
        translator_power: TranslatorPower,
        target_power: TargetPower,
        target_physically_connected: TargetPhysicallyConnected,
        uart_state: Option<UsbUartState>,
    }

    #[init(local = [
        usb_bus: MaybeUninit<UsbBusAllocator<UsbBus>> = MaybeUninit::uninit(),
        delay: MaybeUninit<pico_probe::systick_delay::Delay> = MaybeUninit::uninit(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        let (
            leds,
            probe_usb,
            uart,
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
            Shared { probe_usb, uart },
            Local {
                dap_handler,
                target_vcc,
                translator_power,
                target_power,
                target_physically_connected,
                uart_state: None,
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

    #[task(binds = USBCTRL_IRQ, priority = 2, shared = [ probe_usb, uart ], local = [dap_handler, uart_state, resp_buf: [u8; 64] = [0; 64]])]
    fn on_usb(ctx: on_usb::Context) {
        let mut probe_usb = ctx.shared.probe_usb;
        let mut uart = ctx.shared.uart;
        let dap = ctx.local.dap_handler;
        let resp_buf = ctx.local.resp_buf;
        let uart_state = ctx.local.uart_state;

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

            let uart_state_changed = uart_state.as_ref().map_or(true, |uart_state| !uart_state.eq(probe_usb.serial_line_coding()));
            uart.lock(|uart| {
                if uart_state_changed {
                    *uart_state = Some(UsbUartState::new(probe_usb.serial_line_coding()));
                    if let Some(uart_config) = uart_state.as_ref().unwrap().try_get_uart_config() {
                        uart.configure(uart_config);
                    }
                }

                if buffer_uart_tx_data(probe_usb, uart) > 0 {
                    uart.flush_write_buffer();
                }
            });
        });
    }

    #[task(binds = UART1_IRQ, priority = 3, shared = [probe_usb, uart])]
    fn on_uart(ctx: on_uart::Context) {
        let mut probe_usb = ctx.shared.probe_usb;
        let mut uart = ctx.shared.uart;

        uart.lock(|uart| {
            let mut uart_buf = [0u8; 32];
            let read_size = uart.read(&mut uart_buf);
            probe_usb.lock(|probe_usb| {
                if read_size > 0 {
                    probe_usb.serial_write(&uart_buf[..read_size]);
                }

                buffer_uart_tx_data(probe_usb, uart);
            });

            uart.flush_write_buffer();
        });
    }

    fn buffer_uart_tx_data(probe_usb: &mut pico_probe::usb::ProbeUsb, uart: &mut Uart) -> usize {
        if let Some(mut grant) = uart.try_grant_write(64) {
            let used = probe_usb.serial_read(&mut grant);
            #[cfg(feature = "usb-serial-reboot")]
            if grant.len() >= 4 && &grant[..used] == &0xDABAD000u32.to_be_bytes() {
                rp2040_hal::rom_data::reset_to_usb_boot(0, 0);
            }
            grant.commit(used);
            used
        } else {
            0
        }
    }
}
