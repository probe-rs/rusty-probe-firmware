use rp2040_hal::{
    pac,
    uart::{
        DataBits, Enabled, ReadErrorType, StopBits, UartConfig, UartDevice, UartPeripheral,
        ValidUartPinout,
    },
};
use rtic_monotonics::fugit::{HertzU32, RateExtU32};

pub struct VirtualComPort<D: UartDevice, P: ValidUartPinout<D>> {
    uart: Option<UartPeripheral<Enabled, D, P>>,
    last_baud: HertzU32,
    peripheral_clock: HertzU32,
}

impl<D: UartDevice, P: ValidUartPinout<D>> VirtualComPort<D, P> {
    pub fn new(uart: D, pins: P, resets: &mut pac::RESETS, peripheral_clock: HertzU32) -> Self {
        let initial_config =
            UartConfig::new(115_200_u32.Hz(), DataBits::Eight, None, StopBits::One);
        let mut uart = UartPeripheral::new(uart, pins, resets)
            .enable(initial_config, peripheral_clock)
            .unwrap();
        uart.set_fifos(true);
        // We don't need to set the rx watermark, since the default of halfway full is fine.
        uart.enable_rx_interrupt();

        Self {
            uart: Some(uart),
            last_baud: 115_200_u32.Hz(),
            peripheral_clock: peripheral_clock,
        }
    }

    pub fn update_speed(&mut self, baud: HertzU32, interrupt: rp2040_hal::pac::Interrupt) {
        if self.last_baud == baud {
            return;
        }

        if baud > self.peripheral_clock / 16 || baud < self.peripheral_clock / (16 * 65535) {
            // ignore baud rates that we can't support
            return;
        }

        self.last_baud = baud;

        if let Some(uart) = self.uart.take() {
            let new_config = UartConfig::new(baud, DataBits::Eight, None, StopBits::One);

            let mut new_uart = uart
                .disable()
                .enable(new_config, self.peripheral_clock)
                .unwrap(); // unwrap: we only changed the baud rate, and validated it ourselves
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
