use rp2040_hal::uart;
use rtic_monotonics::rp2040::fugit::HertzU32;

enum Device<D: uart::UartDevice, P: uart::ValidUartPinout<D>> {
    Disabled(uart::UartPeripheral<uart::Disabled, D, P>),
    Enabled(uart::UartPeripheral<uart::Enabled, D, P>)
}

pub use uart::UartConfig as Config;

const WRITE_BUFFER_SIZE: usize = 256;
static WRITE_BUFFER: bbqueue::BBBuffer<WRITE_BUFFER_SIZE> = bbqueue::BBBuffer::new();

pub struct Uart<D: uart::UartDevice, P: uart::ValidUartPinout<D>> {
    device: Option<Device<D, P>>,
    peripheral_freq: HertzU32,
    write_buffer_consumer: bbqueue::Consumer<'static, WRITE_BUFFER_SIZE>,
    write_buffer_producer: bbqueue::Producer<'static, WRITE_BUFFER_SIZE>,
}

impl<D: uart::UartDevice, P: uart::ValidUartPinout<D>> Uart<D, P> {
    pub fn new(device: D, pinout: P, peripheral_freq: HertzU32, resets: &mut rp2040_hal::pac::RESETS) -> Self {
        let (producer, consumer) = WRITE_BUFFER.try_split().unwrap();
        let device = uart::UartPeripheral::new(device, pinout, resets);
        Self {
            device: Some(Device::Disabled(device)),
            peripheral_freq,
            write_buffer_consumer: consumer,
            write_buffer_producer: producer,
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

    pub fn try_grant_write(&mut self, max_size: usize) -> Option<bbqueue::GrantW<'static, WRITE_BUFFER_SIZE>> {
        self.write_buffer_producer.grant_max_remaining(max_size).ok()
    }

    pub fn flush_write_buffer(&mut self) {
        if let Some(Device::Enabled(uart)) = self.device.as_mut() {
            if let Ok(grant) = self.write_buffer_consumer.split_read() {
                let written = uart.write_raw(grant.bufs().0).unwrap_or(&[]);
                let mut used = written.len();
                if written.len() == grant.bufs().0.len() {
                    let written = uart.write_raw(grant.bufs().1).unwrap_or(&[]);
                    used += written.len();
                }

                if used < grant.combined_len() {
                    uart.enable_tx_interrupt();
                } else {
                    uart.disable_tx_interrupt();
                }
                grant.release(used);
            }
        }
    }
}