use rp2040_hal::uart;
use rtic_monotonics::rp2040::fugit::HertzU32;

enum Device<D: uart::UartDevice, P: uart::ValidUartPinout<D>> {
    Disabled(uart::UartPeripheral<uart::Disabled, D, P>),
    Enabled(uart::UartPeripheral<uart::Enabled, D, P>)
}

pub use uart::UartConfig as Config;

pub struct Uart<D: uart::UartDevice, P: uart::ValidUartPinout<D>> {
    device: Option<Device<D, P>>,
    peripheral_freq: HertzU32,
}

impl<D: uart::UartDevice, P: uart::ValidUartPinout<D>> Uart<D, P> {
    pub fn new(device: D, pinout: P, peripheral_freq: HertzU32, resets: &mut rp2040_hal::pac::RESETS) -> Self {
        let device = uart::UartPeripheral::new(device, pinout, resets);
        Self {
            device: Some(Device::Disabled(device)),
            peripheral_freq,
        }
    }

    pub fn configure(&mut self, config: Config) {
        let disabled_device = match self.device.take().unwrap() {
            Device::Disabled(device) => device,
            Device::Enabled(device) => device.disable(),
        };

        let mut device = disabled_device.enable(config, self.peripheral_freq).expect("failed to enable the uart peripheral");
        device.set_fifos(true);
        device.enable_rx_interrupt();
        self.device = Some(Device::Enabled(device));
    }

    pub fn read(&self, data: &mut[u8]) -> usize {
        if let Some(Device::Enabled(uart)) = self.device.as_ref() {
            uart.read_raw(data).unwrap_or(0)
        } else {
            0
        }
    }
}