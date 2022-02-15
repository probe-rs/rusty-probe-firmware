use dap_rs::{swj::Swj, *};
use embedded_hal::{
    blocking::delay::DelayUs,
    digital::v2::{InputPin, OutputPin},
};

pub trait InputToOutput<PO: OutputPin> {
    fn to_output(self) -> PO;
}

pub trait OutputToInput<PI: InputPin> {
    fn to_input(self) -> PI;
}
pub struct Context {
    max_frequency: u32,
    cpu_frequency: u32,
}

impl dap::DapContext for Context {
    fn high_impedance_mode(&mut self) {
        todo!()
    }
}

impl Context {
    pub fn from_pins() -> Self {
        todo!()
    }
}

impl swj::Swj for Context {
    fn pins(&mut self, output: swj::Pins, mask: swj::Pins, wait_us: u32) -> u8 {
        todo!()
    }

    fn sequence(&mut self, data: &[u8], nbits: usize) {
        todo!()
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.max_frequency = max_frequency;

        true
    }
}

pub struct Leds {}

impl dap::DapLeds for Leds {
    fn react_to_host_status(&mut self, _host_status: dap::HostStatus) {}
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

pub struct Swd(Context);

impl swd::Swd<Context> for Swd {
    const AVAILABLE: bool = true;

    fn new(context: Context) -> Self {
        todo!()
    }

    fn release(self) -> Context {
        todo!()
    }

    fn configure(&mut self, period: swd::TurnaroundPeriod, data_phase: swd::DataPhase) -> bool {
        todo!()
    }

    fn read_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister) -> swd::Result<u32> {
        todo!()
    }

    fn write_inner(&mut self, apndp: swd::APnDP, a: swd::DPRegister, data: u32) -> swd::Result<()> {
        todo!()
    }

    fn idle_low(&mut self) {
        todo!()
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        self.0.set_clock(max_frequency)
    }
}

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

pub struct Wait {}

impl DelayUs<u32> for Wait {
    fn delay_us(&mut self, us: u32) {
        todo!()
    }
}

pub fn create_dap(
    version_string: &'static str,
) -> dap::Dap<'static, Context, Leds, Wait, Jtag, Swd, Swo> {
    todo!()
}
