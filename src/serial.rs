use rp2040_hal::{
    pac,
    uart::{
        DataBits as UartDataBits, Enabled, Parity as UartParity, ReadErrorType,
        StopBits as UartStopBits, UartConfig, UartDevice, UartPeripheral, ValidUartPinout,
    },
};
use rtic_monotonics::fugit::{HertzU32, RateExtU32};
use usbd_serial::{LineCoding, ParityType, StopBits as UsbStopBits};

#[derive(PartialEq, Clone, Copy, defmt::Format)]
pub enum DataBits {
    Five,
    Six,
    Seven,
    Eight,
}

impl From<u8> for DataBits {
    fn from(other: u8) -> Self {
        match other {
            5 => Self::Five,
            6 => Self::Six,
            7 => Self::Seven,
            _ => Self::Eight,
        }
    }
}

impl Into<UartDataBits> for DataBits {
    fn into(self) -> UartDataBits {
        match self {
            Self::Five => UartDataBits::Five,
            Self::Six => UartDataBits::Six,
            Self::Seven => UartDataBits::Seven,
            Self::Eight => UartDataBits::Eight,
        }
    }
}

#[derive(PartialEq, Clone, Copy, defmt::Format)]
pub enum Parity {
    None,
    Even,
    Odd,
    // The RP2040 serial port does not support Mark or Space parity, so we map them to None
}

impl From<ParityType> for Parity {
    fn from(other: ParityType) -> Self {
        match other {
            ParityType::None => Self::None,
            ParityType::Even => Self::Even,
            ParityType::Odd => Self::Odd,
            _ => Self::None,
        }
    }
}

impl Into<Option<UartParity>> for Parity {
    fn into(self) -> Option<UartParity> {
        match self {
            Self::None => None,
            Self::Even => Some(UartParity::Even),
            Self::Odd => Some(UartParity::Odd),
        }
    }
}

#[derive(PartialEq, Clone, Copy, defmt::Format)]
pub enum StopBits {
    One,
    Two,
    // The RP2040 serial port does not support 1.5 stop bits, so we map that to two.
}

impl From<UsbStopBits> for StopBits {
    fn from(other: UsbStopBits) -> Self {
        match other {
            UsbStopBits::One => Self::One,
            UsbStopBits::OnePointFive => Self::Two,
            UsbStopBits::Two => Self::Two,
        }
    }
}

impl Into<UartStopBits> for StopBits {
    fn into(self) -> UartStopBits {
        match self {
            Self::One => UartStopBits::One,
            Self::Two => UartStopBits::Two,
        }
    }
}

#[derive(PartialEq, Clone, Copy, defmt::Format)]
pub struct SerialPortConfig {
    pub baud: HertzU32,
    pub data_bits: DataBits,
    pub parity: Parity,
    pub stop_bits: StopBits,
}

impl SerialPortConfig {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn baud(mut self, baud: HertzU32) -> Self {
        self.baud = baud;
        self
    }
    pub fn data_bits(mut self, data_bits: DataBits) -> Self {
        self.data_bits = data_bits;
        self
    }
    pub fn parity(mut self, parity: Parity) -> Self {
        self.parity = parity;
        self
    }
    pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
        self.stop_bits = stop_bits;
        self
    }
}

impl Default for SerialPortConfig {
    fn default() -> Self {
        Self {
            baud: 115_200_u32.Hz(),
            data_bits: DataBits::Eight,
            parity: Parity::None,
            stop_bits: StopBits::One,
        }
    }
}

impl From<&LineCoding> for SerialPortConfig {
    fn from(other: &LineCoding) -> Self {
        Self {
            baud: other.data_rate().Hz(),
            data_bits: other.data_bits().into(),
            parity: other.parity_type().into(),
            stop_bits: other.stop_bits().into(),
        }
    }
}

impl Into<UartConfig> for SerialPortConfig {
    fn into(self) -> UartConfig {
        UartConfig::new(
            self.baud,
            self.data_bits.into(),
            self.parity.into(),
            self.stop_bits.into(),
        )
    }
}

pub struct VirtualComPort<D: UartDevice, P: ValidUartPinout<D>> {
    uart: Option<UartPeripheral<Enabled, D, P>>,
    last_config: SerialPortConfig,
    peripheral_clock: HertzU32,
}

impl<D: UartDevice, P: ValidUartPinout<D>> VirtualComPort<D, P> {
    pub fn new(uart: D, pins: P, resets: &mut pac::RESETS, peripheral_clock: HertzU32) -> Self {
        let initial_config = SerialPortConfig::default();
        let mut uart = UartPeripheral::new(uart, pins, resets)
            .enable(initial_config.into(), peripheral_clock)
            .unwrap();
        uart.set_fifos(true);
        // We don't need to set the rx watermark, since the default of halfway full is fine.
        uart.enable_rx_interrupt();

        Self {
            uart: Some(uart),
            last_config: initial_config,
            peripheral_clock: peripheral_clock,
        }
    }

    pub fn update_coding(&mut self, coding: &LineCoding, interrupt: rp2040_hal::pac::Interrupt) {
        let config = coding.into();

        if self.last_config == config {
            return;
        }

        if config.baud > self.peripheral_clock / 16
            || config.baud < self.peripheral_clock / (16 * 65535)
        {
            // ignore baud rates that we can't support
            return;
        }

        self.last_config = config;

        if let Some(uart) = self.uart.take() {
            let mut new_uart = uart
                .disable()
                .enable(config.into(), self.peripheral_clock)
                .unwrap(); // unwrap: we validated the baud rate ourselves
            new_uart.set_fifos(true);
            new_uart.enable_rx_interrupt();
            self.uart = Some(new_uart);
            rtic::pend(interrupt);
        }
    }

    pub fn interrupt<const N: usize>(
        &mut self,
        queues: &mut crate::setup::VCPQueues<'static, N>,
        interrupt: rp2040_hal::pac::Interrupt,
    ) -> Result<(), ReadErrorType> {
        if let Some(uart) = self.uart.as_mut() {
            let mut buf = [0u8; 32];
            let len = uart.read_raw(&mut buf).or_else(|e| match e {
                nb::Error::WouldBlock => Ok(0),
                nb::Error::Other(err) => Err(err.err_type),
            })?;

            let mut slice = &buf[0..len];

            while slice.len() > 0 {
                // We want the queue to act like a ring buffer, so if there isn't enough space to
                // put the whole slice onto the producer queue, we write what we can and drop some
                // old data from the consumer side.
                match queues.uart_to_usb_p.grant_max_remaining(slice.len()) {
                    Ok(mut g) => {
                        let to_copy = g.buf().len().min(slice.len());
                        g.buf()[0..to_copy].copy_from_slice(&slice[0..to_copy]);
                        slice = &slice[to_copy..];
                        g.commit(to_copy);
                        // Kick the USB processing in case the USB peripheral stopped triggering
                        // the write fifo interrupts due to no data writes.
                        rtic::pend(interrupt);
                    }
                    Err(bbqueue::Error::InsufficientSize) => {
                        // throw away this many bytes from the other end
                        queues.uart_to_usb_c.read().unwrap().release(slice.len());
                    }
                    Err(_) => {
                        unreachable!();
                    }
                }
            }

            match queues.usb_to_uart_c.read() {
                Ok(q) => {
                    let buf = if q.buf().len() > 32 {
                        &q.buf()[0..32]
                    } else {
                        &q.buf()
                    };

                    let len = uart
                        .write_raw(buf)
                        .map(|rem| buf.len() - rem.len())
                        .or_else(|e| match e {
                            nb::Error::WouldBlock => Ok(0),
                            nb::Error::Other(_) => Ok(0), // write_raw returns Infallible
                        })?;
                    q.release(len);

                    if len > 0 {
                        uart.enable_tx_interrupt();
                    }
                }
                Err(bbqueue::Error::InsufficientSize) => {
                    // Nothing in the USB to UART queue, so no point in spamming the CPU with
                    // interrupts for an empty TX FIFO.
                    uart.disable_tx_interrupt();
                }
                Err(_) => {
                    // The only other error is another read grant in progress, and the interrupt
                    // priorities prevent that.
                    unreachable!();
                }
            }
        }

        Ok(())
    }
}
