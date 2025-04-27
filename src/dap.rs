use crate::{
    setup::{
        DirTckSwclkPin, DirTdiPin, DirTdoSwoPin, DirTmsSwdioPin, ResetPin, TckSwclkPin, TdiPin,
        TdoSwoPin, TmsSwdioPin,
    },
    systick_delay::Delay,
};
use dap_rs::{jtag::TapConfig, swj::Dependencies, *};
use defmt::trace;
use embedded_hal::{
    delay::DelayNs,
    digital::{OutputPin, PinState},
};

pub struct Context {
    max_frequency: u32,
    cpu_frequency: u32,
    cycles_per_us: u32,
    half_period_ticks: u32,
    delay: &'static Delay,
    tms_swdio: TmsSwdioPin,
    tck_swclk: TckSwclkPin,
    tdo_swo: TdoSwoPin,
    tdi: TdiPin,
    nreset: ResetPin,
    dir_tms_swdio: DirTmsSwdioPin,
    dir_tck_swclk: DirTckSwclkPin,
    dir_tdo_swo: DirTdoSwoPin,
    dir_tdi: DirTdiPin,
    swd_config: swd::Config,
    jtag_config: jtag::Config,
}

impl defmt::Format for Context {
    fn format(&self, f: defmt::Formatter) {
        // format the bitfields of the register as struct fields
        defmt::write!(
           f,
           "Context {{ max_frequency: {}, cpu_frequency: {}, cycles_per_us: {}, half_period_ticks: {} }}",
            self.max_frequency,
            self.cpu_frequency,
            self.cycles_per_us,
            self.half_period_ticks,
        )
    }
}

impl core::fmt::Debug for Context {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Context")
            .field("max_frequency", &self.max_frequency)
            .field("cpu_frequency", &self.cpu_frequency)
            .field("cycles_per_us", &self.cycles_per_us)
            .field("half_period_ticks", &self.half_period_ticks)
            .finish()
    }
}

impl Context {
    fn tms_swdio_to_input(&mut self) {
        defmt::trace!("TMS/SWDIO -> input");
        self.dir_tms_swdio.set_low().ok();
        self.tms_swdio.into_input();
    }

    fn tms_swdio_to_output(&mut self) {
        defmt::trace!("TMS/SWDIO -> output");
        self.tms_swdio.into_output_in_state(PinState::High);
        self.dir_tms_swdio.set_high().ok();
    }

    fn tck_swclk_to_input(&mut self) {
        defmt::trace!("TCK/SWCLK -> input");
        self.dir_tck_swclk.set_low().ok();
        self.tck_swclk.into_input();
    }

    fn tck_swclk_to_output(&mut self) {
        defmt::trace!("TCK/SWCLK -> output");
        self.tck_swclk.into_output_in_state(PinState::High);
        self.dir_tck_swclk.set_high().ok();
    }

    fn tdo_swo_to_input(&mut self) {
        defmt::trace!("TDO/SWO -> input");
        self.dir_tdo_swo.set_low().ok();
        self.tdo_swo.into_input();
    }

    fn tdo_swo_to_output(&mut self) {
        defmt::trace!("TDO/SWO -> output");
        self.tdo_swo.into_output_in_state(PinState::High);
        self.dir_tdo_swo.set_high().ok();
    }

    fn tdi_to_input(&mut self) {
        defmt::trace!("TDI -> input");
        self.dir_tdi.set_low().ok();
        self.tdi.into_input();
    }

    fn tdi_to_output(&mut self) {
        defmt::trace!("TDI -> output");
        self.tdi.into_output_in_state(PinState::High);
        self.dir_tdi.set_high().ok();
    }

    fn from_pins(
        tms_swdio: TmsSwdioPin,
        tck_swclk: TckSwclkPin,
        tdo_swo: TdoSwoPin,
        tdi: TdiPin,
        nreset: ResetPin,
        mut dir_tms_swdio: DirTmsSwdioPin,
        mut dir_tck_swclk: DirTckSwclkPin,
        mut dir_tdo_swo: DirTdoSwoPin,
        mut dir_tdi: DirTdiPin,
        cpu_frequency: u32,
        delay: &'static Delay,
        scan_chain: &'static mut [TapConfig],
    ) -> Self {
        dir_tms_swdio.set_low().ok();
        dir_tck_swclk.set_low().ok();
        dir_tdo_swo.set_low().ok();
        dir_tdi.set_low().ok();
        defmt::trace!("TMS/SWCLK -> input");
        defmt::trace!("TCK/SWDIO -> input");
        defmt::trace!("TDO/SDO -> input");
        defmt::trace!("TDI -> input");

        let max_frequency = 100_000;
        let half_period_ticks = cpu_frequency / max_frequency / 2;
        Context {
            max_frequency,
            cpu_frequency,
            cycles_per_us: cpu_frequency / 1_000_000,
            half_period_ticks,
            delay,
            tms_swdio,
            tck_swclk,
            tdo_swo,
            tdi,
            nreset,
            dir_tms_swdio,
            dir_tck_swclk,
            dir_tdo_swo,
            dir_tdi,
            swd_config: swd::Config::default(),
            jtag_config: jtag::Config::new(scan_chain),
        }
    }
}

impl swj::Dependencies<Swd, Jtag> for Context {
    fn process_swj_pins(&mut self, output: swj::Pins, mask: swj::Pins, wait_us: u32) -> swj::Pins {
        if mask.contains(swj::Pins::SWCLK) {
            self.tck_swclk_to_output();
            if output.contains(swj::Pins::SWCLK) {
                self.tck_swclk.set_high();
            } else {
                self.tck_swclk.set_low();
            }
        }

        if mask.contains(swj::Pins::SWDIO) {
            self.tms_swdio_to_output();
            if output.contains(swj::Pins::SWDIO) {
                self.tms_swdio.set_high();
            } else {
                self.tms_swdio.set_low();
            }
        }

        if mask.contains(swj::Pins::TDO) {
            self.tdo_swo_to_output();
            if output.contains(swj::Pins::TDO) {
                self.tdo_swo.set_high();
            } else {
                self.tdo_swo.set_low();
            }
        }

        if mask.contains(swj::Pins::TDI) {
            self.tdi_to_output();
            if output.contains(swj::Pins::TDI) {
                self.tdi.set_high();
            } else {
                self.tdi.set_low();
            }
        }

        if mask.contains(swj::Pins::NRESET) {
            if output.contains(swj::Pins::NRESET) {
                // "open drain disconnect"
                self.nreset.into_input();
            } else {
                self.nreset.into_output_in_state(PinState::Low);
            }
        }

        // Delay until desired output state or timeout.
        let mut last = self.delay.get_current();
        for _ in 0..wait_us {
            last = self.delay.delay_ticks_from_last(self.cycles_per_us, last);

            // If a pin is selected, make sure its output equals the desired output state, or else
            // continue waiting.
            let swclk_not_in_desired_state = mask.contains(swj::Pins::SWCLK)
                && output.contains(swj::Pins::SWCLK) != self.tck_swclk.is_high();
            let swdio_not_in_desired_state = mask.contains(swj::Pins::SWDIO)
                && output.contains(swj::Pins::SWDIO) != self.tms_swdio.is_high();
            let tdo_not_in_desired_state = mask.contains(swj::Pins::TDO)
                && output.contains(swj::Pins::TDO) != self.tdo_swo.is_high();
            let tdi_not_in_desired_state = mask.contains(swj::Pins::TDI)
                && output.contains(swj::Pins::TDI) != self.tdi.is_high();
            let nreset_not_in_desired_state = mask.contains(swj::Pins::NRESET)
                && output.contains(swj::Pins::NRESET) != self.nreset.is_high();

            if swclk_not_in_desired_state
                || swdio_not_in_desired_state
                || tdo_not_in_desired_state
                || tdi_not_in_desired_state
                || nreset_not_in_desired_state
            {
                continue;
            }

            break;
        }

        // Read back input state, wait 1us to have inputs stabilize.
        self.tck_swclk_to_input();
        self.tms_swdio_to_input();
        self.tdo_swo_to_input();
        self.tdi_to_input();
        self.nreset.into_input();

        self.delay.delay_ticks_from_last(self.cycles_per_us, last);

        let mut ret = swj::Pins::empty();
        ret.set(swj::Pins::SWCLK, self.tck_swclk.is_high());
        ret.set(swj::Pins::SWDIO, self.tms_swdio.is_high());
        ret.set(swj::Pins::TDO, self.tdo_swo.is_high());
        ret.set(swj::Pins::TDI, self.tdi.is_high());
        ret.set(swj::Pins::NRESET, self.nreset.is_high());

        trace!(
            "Running SWJ_pins: mask {:08b}, output: {:08b}, read: {:08b}",
            mask.bits(),
            output.bits(),
            ret.bits()
        );

        ret
    }

    fn process_swj_sequence(&mut self, data: &[u8], mut bits: usize) {
        self.tck_swclk_to_output();
        self.tms_swdio_to_output();

        let half_period_ticks = self.half_period_ticks;
        let mut last = self.delay.get_current();
        last = self.delay.delay_ticks_from_last(half_period_ticks, last);

        trace!("Running SWJ sequence: {:08b}, len = {}", data, bits);
        for byte in data {
            let mut byte = *byte;
            let frame_bits = core::cmp::min(bits, 8);
            for _ in 0..frame_bits {
                let bit = byte & 1;
                byte >>= 1;
                if bit != 0 {
                    self.tms_swdio.set_high();
                } else {
                    self.tms_swdio.set_low();
                }
                self.tck_swclk.set_low();
                last = self.delay.delay_ticks_from_last(half_period_ticks, last);
                self.tck_swclk.set_high();
                last = self.delay.delay_ticks_from_last(half_period_ticks, last);
            }
            bits -= frame_bits;
        }
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        trace!("Running SWJ clock");
        if max_frequency < self.cpu_frequency {
            self.max_frequency = max_frequency;
            self.half_period_ticks = self.cpu_frequency / self.max_frequency / 2;
            trace!("  freq = {}", max_frequency);
            trace!("  half_period_ticks = {}", self.half_period_ticks);
            true
        } else {
            false
        }
    }

    fn high_impedance_mode(&mut self) {
        self.tms_swdio_to_input();
        self.tck_swclk_to_input();
        self.tdo_swo_to_input();
        self.tdi_to_input();
        self.nreset.into_input();
    }

    fn swd_config(&mut self) -> &mut swd::Config {
        &mut self.swd_config
    }

    fn jtag_config(&mut self) -> &mut jtag::Config {
        &mut self.jtag_config
    }
}

pub struct Jtag(Context);

impl From<Jtag> for Context {
    fn from(value: Jtag) -> Self {
        value.0
    }
}

impl From<Context> for Jtag {
    fn from(mut value: Context) -> Self {
        value.nreset.into_output_in_state(PinState::High);
        value.tms_swdio_to_output();
        value.tck_swclk_to_output();
        value.tdi_to_output();

        value.tdo_swo_to_input();

        Self(value)
    }
}

impl Jtag {
    fn shift_jtag(&mut self, tms: bool, tdi: u32, num_bits: usize) -> u32 {
        if tms {
            self.0.tms_swdio.set_high();
        } else {
            self.0.tms_swdio.set_low();
        }

        let mut last = self.0.delay.get_current();
        let half_period_ticks = self.0.half_period_ticks;

        let mut tdo = 0;
        for i in 0..num_bits {
            if tdi & (1 << i) != 0 {
                self.0.tdi.set_high();
            } else {
                self.0.tdi.set_low();
            }

            self.0.tck_swclk.set_low();
            last = self.0.delay.delay_ticks_from_last(half_period_ticks, last);

            tdo |= (self.0.tdo_swo.is_high() as u32) << i;

            self.0.tck_swclk.set_high();
            last = self.0.delay.delay_ticks_from_last(half_period_ticks, last);
        }
        tdo
    }
}

impl jtag::Jtag<Context> for Jtag {
    const AVAILABLE: bool = true;

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.process_swj_clock(max_frequency)
    }

    fn config(&mut self) -> &mut jtag::Config {
        &mut self.0.jtag_config
    }

    fn sequence(&mut self, info: jtag::SequenceInfo, tdi: &[u8], mut rxbuf: &mut [u8]) {
        self.0.tms_swdio_to_output();
        self.0.tdi_to_output();
        self.0.tdo_swo_to_input();
        self.0.tck_swclk_to_output();

        let mut n_bits = info.n_bits;
        for &byte in tdi {
            let bits = n_bits.min(8) as usize;
            n_bits -= bits as u8;
            let tdo = self.shift_jtag(info.tms, byte as u32, bits);
            if info.capture && !rxbuf.is_empty() {
                rxbuf[0] = tdo as u8;
                rxbuf = &mut rxbuf[1..];
            }
        }
    }

    fn tms_sequence(&mut self, tms: &[bool]) {
        self.0.tms_swdio_to_output();
        self.0.tck_swclk_to_output();

        let mut last = self.0.delay.get_current();
        let half_period_ticks = self.0.half_period_ticks;

        for &bit in tms {
            if bit {
                self.0.tms_swdio.set_high();
            } else {
                self.0.tms_swdio.set_low();
            }

            self.0.tck_swclk.set_low();
            last = self.0.delay.delay_ticks_from_last(half_period_ticks, last);
            self.0.tck_swclk.set_high();
            last = self.0.delay.delay_ticks_from_last(half_period_ticks, last);
        }
    }
}

#[derive(Debug, defmt::Format)]
pub struct Swd(Context);

impl From<Swd> for Context {
    fn from(value: Swd) -> Self {
        value.0
    }
}

impl From<Context> for Swd {
    fn from(mut value: Context) -> Self {
        value.tms_swdio_to_output();
        value.tck_swclk_to_output();
        value.tdo_swo_to_input();
        value.tdi_to_input();
        value.nreset.into_input();

        Self(value)
    }
}

impl swd::Swd<Context> for Swd {
    const AVAILABLE: bool = true;

    fn read_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister) -> swd::Result<u32> {
        trace!("SWD read, apndp: {}, addr: {}", apndp, a,);
        // Send request
        let req = swd::make_request(apndp, swd::RnW::R, a);
        trace!("SWD tx request");
        self.tx8(req);

        trace!("SWD rx ack");
        // Read ack, 1 clock for turnaround and 3 for ACK
        let ack = self.rx4() >> 1;

        match swd::Ack::try_ok(ack as u8) {
            Ok(_) => trace!("    ack ok"),
            Err(e) => {
                trace!("    ack error: {}", e);
                // On non-OK ACK, target has released the bus but
                // is still expecting a turnaround clock before
                // the next request, and we need to take over the bus.
                self.tx8(0);
                return Err(e);
            }
        }

        // Read data and parity
        trace!("SWD rx data");
        let (data, parity) = self.read_data();

        // Turnaround + trailing
        let mut last = self.0.delay.get_current();
        self.read_bit(&mut last);
        self.tx8(0); // Drive the SWDIO line to 0 to not float

        if parity as u8 == (data.count_ones() as u8 & 1) {
            trace!("    data: 0x{:x}", data);
            Ok(data)
        } else {
            Err(swd::Error::BadParity)
        }
    }

    fn write_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister, data: u32) -> swd::Result<()> {
        trace!(
            "SWD write, apndp: {}, addr: {}, data: 0x{:x}",
            apndp,
            a,
            data
        );

        // Send request
        let req = swd::make_request(apndp, swd::RnW::W, a);
        trace!("SWD tx request");
        self.tx8(req);

        // Read ack, 1 clock for turnaround and 3 for ACK and 1 for turnaround
        trace!("SWD rx ack");
        let ack = (self.rx5() >> 1) & 0b111;
        match swd::Ack::try_ok(ack as u8) {
            Ok(_) => trace!("    ack ok"),
            Err(e) => {
                trace!("    ack err: {}, data: {:b}", e, ack);
                // On non-OK ACK, target has released the bus but
                // is still expecting a turnaround clock before
                // the next request, and we need to take over the bus.
                self.tx8(0);
                return Err(e);
            }
        }

        // Send data and parity
        trace!("SWD tx data");
        let parity = data.count_ones() & 1 == 1;
        self.send_data(data, parity);

        // Send trailing idle
        self.tx8(0);

        Ok(())
    }

    fn write_sequence(&mut self, mut num_bits: usize, data: &[u8]) -> swd::Result<()> {
        self.0.tms_swdio_to_output();
        let mut last = self.0.delay.get_current();

        for b in data {
            let bit_count = core::cmp::min(num_bits, 8);
            for i in 0..bit_count {
                self.write_bit((b >> i) & 0x1, &mut last);
            }
            num_bits -= bit_count;
        }

        Ok(())
    }

    fn read_sequence(&mut self, mut num_bits: usize, data: &mut [u8]) -> swd::Result<()> {
        self.0.tms_swdio_to_input();
        let mut last = self.0.delay.get_current();

        for b in data {
            let bit_count = core::cmp::min(num_bits, 8);
            for i in 0..bit_count {
                let bit = self.read_bit(&mut last);
                *b |= bit << i;
            }
            num_bits -= bit_count;
        }

        Ok(())
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        trace!("SWD set clock: freq = {}", max_frequency);
        self.0.process_swj_clock(max_frequency)
    }

    fn config(&mut self) -> &mut swd::Config {
        &mut self.0.swd_config
    }
}

impl Swd {
    fn tx8(&mut self, mut data: u8) {
        self.0.tms_swdio_to_output();

        let mut last = self.0.delay.get_current();

        for _ in 0..8 {
            self.write_bit(data & 1, &mut last);
            data >>= 1;
        }
    }

    fn rx4(&mut self) -> u8 {
        self.0.tms_swdio_to_input();

        let mut data = 0;
        let mut last = self.0.delay.get_current();

        for i in 0..4 {
            data |= (self.read_bit(&mut last) & 1) << i;
        }

        data
    }

    fn rx5(&mut self) -> u8 {
        self.0.tms_swdio_to_input();

        let mut last = self.0.delay.get_current();

        let mut data = 0;

        for i in 0..5 {
            data |= (self.read_bit(&mut last) & 1) << i;
        }

        data
    }

    fn send_data(&mut self, mut data: u32, parity: bool) {
        self.0.tms_swdio_to_output();

        let mut last = self.0.delay.get_current();

        for _ in 0..32 {
            self.write_bit((data & 1) as u8, &mut last);
            data >>= 1;
        }

        self.write_bit(parity as u8, &mut last);
    }

    fn read_data(&mut self) -> (u32, bool) {
        self.0.tms_swdio_to_input();

        let mut data = 0;

        let mut last = self.0.delay.get_current();

        for i in 0..32 {
            data |= (self.read_bit(&mut last) as u32 & 1) << i;
        }

        let parity = self.read_bit(&mut last) != 0;

        (data, parity)
    }

    #[inline(always)]
    fn write_bit(&mut self, bit: u8, last: &mut u32) {
        if bit != 0 {
            self.0.tms_swdio.set_high();
        } else {
            self.0.tms_swdio.set_low();
        }

        let half_period_ticks = self.0.half_period_ticks;

        self.0.tck_swclk.set_low();
        *last = self.0.delay.delay_ticks_from_last(half_period_ticks, *last);
        self.0.tck_swclk.set_high();
        *last = self.0.delay.delay_ticks_from_last(half_period_ticks, *last);
    }

    #[inline(always)]
    fn read_bit(&mut self, last: &mut u32) -> u8 {
        let half_period_ticks = self.0.half_period_ticks;

        self.0.tck_swclk.set_low();
        *last = self.0.delay.delay_ticks_from_last(half_period_ticks, *last);
        let bit = self.0.tms_swdio.is_high() as u8;
        self.0.tck_swclk.set_high();
        *last = self.0.delay.delay_ticks_from_last(half_period_ticks, *last);

        bit
    }
}

#[derive(Debug, defmt::Format)]
pub struct Swo {}

impl swo::Swo for Swo {
    fn set_transport(&mut self, _transport: swo::SwoTransport) {}

    fn set_mode(&mut self, _mode: swo::SwoMode) {}

    fn set_baudrate(&mut self, _baudrate: u32) -> u32 {
        0
    }

    fn set_control(&mut self, _control: swo::SwoControl) {}

    fn polling_data(&mut self, _buf: &mut [u8]) -> u32 {
        0
    }

    fn streaming_data(&mut self) {}

    fn is_active(&self) -> bool {
        false
    }

    fn bytes_available(&self) -> u32 {
        0
    }

    fn buffer_size(&self) -> u32 {
        0
    }

    fn support(&self) -> swo::SwoSupport {
        swo::SwoSupport {
            uart: false,
            manchester: false,
        }
    }

    fn status(&mut self) -> swo::SwoStatus {
        swo::SwoStatus {
            active: false,
            trace_error: false,
            trace_overrun: false,
            bytes_available: 0,
        }
    }
}

pub struct Wait {
    delay: &'static Delay,
}

impl Wait {
    pub fn new(delay: &'static Delay) -> Self {
        Wait { delay }
    }
}

impl DelayNs for Wait {
    fn delay_ns(&mut self, ns: u32) {
        self.delay.delay_us((ns / 1_000).max(1));
    }
}

#[inline(always)]
pub fn create_dap(
    version_string: &'static str,
    tms_swdio: TmsSwdioPin,
    tck_swclk: TckSwclkPin,
    tdo_swo: TdoSwoPin,
    tdi: TdiPin,
    nreset: ResetPin,
    dir_tms_swdio: DirTmsSwdioPin,
    dir_tck_swclk: DirTckSwclkPin,
    dir_tdo_swo: DirTdoSwoPin,
    dir_tdi: DirTdiPin,
    cpu_frequency: u32,
    delay: &'static Delay,
    scan_chain: &'static mut [TapConfig],
    leds: crate::leds::HostStatusToken,
) -> crate::setup::DapHandler {
    let context = Context::from_pins(
        tms_swdio,
        tck_swclk,
        tdo_swo,
        tdi,
        nreset,
        dir_tms_swdio,
        dir_tck_swclk,
        dir_tdo_swo,
        dir_tdi,
        cpu_frequency,
        delay,
        scan_chain,
    );
    let wait = Wait::new(delay);
    let swo = None;

    defmt::info!("Making dap interface with context: {}", context);

    dap::Dap::new(context, leds, wait, swo, version_string)
}
