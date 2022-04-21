use crate::systick_delay::Delay;
use dap_rs::{swj::Swj, *};
use defmt::*;
use embedded_hal::{
    blocking::delay::DelayUs,
    digital::v2::{InputPin, OutputPin, PinState},
};
use rp2040_hal::gpio::DynPin;

pub struct Context {
    max_frequency: u32,
    cpu_frequency: u32,
    cycles_per_us: u32,
    half_period_ticks: u32,
    delay: &'static Delay,
    swdio: DynPin,
    swclk: DynPin,
    nreset: DynPin,
    dir_swdio: DynPin,
    dir_swclk: DynPin,
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

impl dap::DapContext for Context {
    fn high_impedance_mode(&mut self) {
        self.swdio_to_input();
        self.swclk_to_input();
        self.nreset.into_floating_input();
    }
}

impl Context {
    fn swdio_to_input(&mut self) {
        self.swdio.into_pull_down_input();
        self.dir_swdio.set_low().ok();
    }

    fn swdio_to_output(&mut self) {
        self.dir_swdio.set_high().ok();
        self.swdio.into_push_pull_output();
    }

    fn swclk_to_input(&mut self) {
        self.swclk.into_pull_down_input();
        self.dir_swclk.set_low().ok();
    }

    fn swclk_to_output(&mut self) {
        self.dir_swclk.set_high().ok();
        self.swclk.into_push_pull_output();
    }

    fn from_pins(
        swdio: DynPin,
        swclk: DynPin,
        nreset: DynPin,
        mut dir_swdio: DynPin,
        mut dir_swclk: DynPin,
        cpu_frequency: u32,
        delay: &'static Delay,
    ) -> Self {
        dir_swdio.into_push_pull_output();
        dir_swclk.into_push_pull_output();

        dir_swdio.set_low().ok();
        dir_swclk.set_low().ok();

        let max_frequency = 100_000;
        let half_period_ticks = cpu_frequency / max_frequency / 2;
        Context {
            max_frequency,
            cpu_frequency,
            cycles_per_us: cpu_frequency / 1_000_000,
            half_period_ticks,
            delay,
            swdio,
            swclk,
            nreset,
            dir_swdio,
            dir_swclk,
        }
    }
}

impl swj::Swj for Context {
    fn pins(&mut self, output: swj::Pins, mask: swj::Pins, wait_us: u32) -> swj::Pins {
        trace!("Running SWJ_pins");
        if mask.contains(swj::Pins::SWCLK) {
            self.swclk_to_output();
            self.swclk
                .set_state(if output.contains(swj::Pins::SWCLK) {
                    PinState::High
                } else {
                    PinState::Low
                })
                .ok();
        }

        if mask.contains(swj::Pins::SWDIO) {
            self.swdio_to_output();
            self.swdio
                .set_state(if output.contains(swj::Pins::SWDIO) {
                    PinState::High
                } else {
                    PinState::Low
                })
                .ok();
        }

        if mask.contains(swj::Pins::NRESET) {
            if output.contains(swj::Pins::NRESET) {
                // "open drain"
                self.nreset.into_floating_disabled();
            } else {
                self.nreset.into_push_pull_output();
                self.nreset.set_low().ok();
            }
        }

        self.delay.delay_ticks(self.cycles_per_us * wait_us);

        self.swclk_to_input();
        self.swdio_to_input();
        self.nreset.into_floating_input();

        let mut ret = swj::Pins::empty();
        ret.set(swj::Pins::SWCLK, matches!(self.swclk.is_high(), Ok(true)));
        ret.set(swj::Pins::SWDIO, matches!(self.swdio.is_high(), Ok(true)));
        ret.set(swj::Pins::NRESET, matches!(self.nreset.is_high(), Ok(true)));

        ret
    }

    fn sequence(&mut self, data: &[u8], mut bits: usize) {
        trace!("Running SWJ sequence");
        self.swclk_to_output();
        self.swdio_to_output();

        let half_period_ticks = self.half_period_ticks;
        let mut last = self.delay.get_current();
        last = self.delay.delay_ticks_from_last(half_period_ticks, last);

        for byte in data {
            let mut byte = *byte;
            let frame_bits = core::cmp::min(bits, 8);
            for _ in 0..frame_bits {
                let bit = byte & 1;
                byte >>= 1;
                if bit != 0 {
                    self.swdio.set_high().ok();
                } else {
                    self.swdio.set_low().ok();
                }
                self.swclk.set_low().ok();
                last = self.delay.delay_ticks_from_last(half_period_ticks, last);
                self.swclk.set_high().ok();
                last = self.delay.delay_ticks_from_last(half_period_ticks, last);
            }
            bits -= frame_bits;
        }

        self.swclk_to_input();
        self.swdio_to_input();
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
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
}

#[derive(Debug, defmt::Format)]
pub struct Leds {}

impl dap::DapLeds for Leds {
    fn react_to_host_status(&mut self, _host_status: dap::HostStatus) {
        trace!("Running LEDs react to host status");
    }
}

pub struct Jtag(Context);

impl jtag::Jtag<Context> for Jtag {
    const AVAILABLE: bool = false;

    fn new(context: Context) -> Self {
        Jtag(context)
    }

    fn release(self) -> Context {
        self.0
    }

    fn sequences(&mut self, _data: &[u8], _rxbuf: &mut [u8]) -> u32 {
        0
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_clock(max_frequency)
    }
}

#[derive(Debug, defmt::Format)]
pub struct Swd(Context);

impl swd::Swd<Context> for Swd {
    const AVAILABLE: bool = true;

    fn new(mut context: Context) -> Self {
        trace!("Creating SWD");
        context.swclk_to_output();
        context.swdio_to_output();

        Self(context)
    }

    fn release(mut self) -> Context {
        trace!("Releasing SWD");
        self.0.swclk_to_input();
        self.0.swdio_to_input();

        self.0
    }

    fn configure(&mut self, period: swd::TurnaroundPeriod, data_phase: swd::DataPhase) -> bool {
        trace!("SWD configure");
        period == swd::TurnaroundPeriod::Cycles1 && data_phase == swd::DataPhase::NoDataPhase
    }

    fn read_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister) -> swd::Result<u32> {
        trace!("SWD read, apndp: {}, addr: {}", apndp, a,);
        // Send request
        let req = swd::make_request(apndp, swd::RnW::R, a);
        self.tx8(req);

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
        self.tx8(req);

        // Read ack, 1 clock for turnaround and 3 for ACK and 1 for turnaround
        let ack = (self.rx5() >> 1) & 0b111;
        match swd::Ack::try_ok(ack as u8) {
            Ok(_) => trace!("    ack ok"),
            Err(e) => {
                trace!("    ack err: {}", e);
                // On non-OK ACK, target has released the bus but
                // is still expecting a turnaround clock before
                // the next request, and we need to take over the bus.
                self.tx8(0);
                return Err(e);
            }
        }

        // Send data and parity
        let parity = data.count_ones() & 1 == 1;
        self.send_data(data, parity);

        // Send trailing idle
        self.tx8(0);

        Ok(())
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        trace!("SWD set clock: freq = {}", max_frequency);
        self.0.set_clock(max_frequency)
    }
}

impl Swd {
    fn tx8(&mut self, mut data: u8) {
        self.0.swdio_to_output();

        let mut last = self.0.delay.get_current();

        for _ in 0..8 {
            self.write_bit(data & 1, &mut last);
            data >>= 1;
        }
    }

    fn rx4(&mut self) -> u8 {
        self.0.swdio_to_input();

        let mut data = 0;
        let mut last = self.0.delay.get_current();

        for i in 0..4 {
            data |= (self.read_bit(&mut last) & 1) << i;
        }

        data
    }

    fn rx5(&mut self) -> u8 {
        self.0.swdio_to_input();

        let mut last = self.0.delay.get_current();

        let mut data = 0;

        for i in 0..5 {
            data |= (self.read_bit(&mut last) & 1) << i;
        }

        data
    }

    fn send_data(&mut self, mut data: u32, parity: bool) {
        self.0.swdio_to_output();

        let mut last = self.0.delay.get_current();

        for _ in 0..32 {
            self.write_bit((data & 1) as u8, &mut last);
            data >>= 1;
        }

        self.write_bit(parity as u8, &mut last);
    }

    fn read_data(&mut self) -> (u32, bool) {
        self.0.swdio_to_input();

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
            self.0.swdio.set_high().ok();
        } else {
            self.0.swdio.set_low().ok();
        }

        let half_period_ticks = self.0.half_period_ticks;

        self.0.swclk.set_low().ok();
        *last = self.0.delay.delay_ticks_from_last(half_period_ticks, *last);
        self.0.swclk.set_high().ok();
        *last = self.0.delay.delay_ticks_from_last(half_period_ticks, *last);
    }

    #[inline(always)]
    fn read_bit(&mut self, last: &mut u32) -> u8 {
        let half_period_ticks = self.0.half_period_ticks;

        self.0.swclk.set_low().ok();
        *last = self.0.delay.delay_ticks_from_last(half_period_ticks, *last);
        let bit = matches!(self.0.swdio.is_high(), Ok(true)) as u8;
        self.0.swclk.set_high().ok();
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

impl DelayUs<u32> for Wait {
    fn delay_us(&mut self, us: u32) {
        self.delay.delay_us(us);
    }
}

#[inline(always)]
pub fn create_dap(
    version_string: &'static str,
    swdio: DynPin,
    swclk: DynPin,
    nreset: DynPin,
    dir_swdio: DynPin,
    dir_swclk: DynPin,
    cpu_frequency: u32,
    delay: &'static Delay,
) -> dap::Dap<'static, Context, Leds, Wait, Jtag, Swd, Swo> {
    let context = Context::from_pins(
        swdio,
        swclk,
        nreset,
        dir_swdio,
        dir_swclk,
        cpu_frequency,
        delay,
    );
    let leds = Leds {};
    let wait = Wait::new(delay);
    let swo = None;

    defmt::info!("Making dap interface with context: {}", context);

    dap::Dap::from_parts(context, leds, wait, swo, version_string)
}
