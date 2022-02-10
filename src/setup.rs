use rp2040_monotonic::Rp2040Monotonic;
use rp_pico::{
    hal::{
        clocks::init_clocks_and_plls,
        gpio::{
            pin::bank0::{Gpio23, Gpio24, Gpio25},
            Pin, Pins, PullDownInput, PushPullOutput,
        },
        pac,
        watchdog::Watchdog,
        Sio,
    },
    XOSC_CRYSTAL_FREQ,
};

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

pub type LedPin = Pin<Gpio25, PushPullOutput>;

#[inline(always)]
pub fn setup(pac: pac::Peripherals) -> (Rp2040Monotonic, LedPin, Swd) {
    let mut resets = pac.RESETS;
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let _clocks = defmt::unwrap!(init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    )
    .ok());

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

    let led = pins.gpio25.into_push_pull_output();
    let mut io = pins.gpio23.into_push_pull_output();
    io.set_low().ok();
    let mut ck = pins.gpio24.into_push_pull_output();
    ck.set_low().ok();

    let mono = Rp2040Monotonic::new(pac.TIMER);

    (mono, led, Swd { io, ck })
}

pub struct Swd {
    io: Pin<Gpio23, PushPullOutput>,
    ck: Pin<Gpio24, PushPullOutput>,
}

const DELAY: u32 = 100_000;

impl Swd {
    pub fn write(self, req: u8, data: u32) -> Self {
        let Swd { mut io, mut ck } = self;

        Self::send_u8(&mut io, &mut ck, req);

        let (ack, mut io) = Self::read_ack(io, &mut ck);

        defmt::info!("ack: {}", ack);

        for b in data.to_le_bytes() {
            Self::send_u8(&mut io, &mut ck, b);
        }

        let parity = data.count_ones() as u8;

        Self::send_u8(&mut io, &mut ck, parity & 1);

        Swd { io, ck }
    }

    fn read_ack(
        io: Pin<Gpio23, PushPullOutput>,
        ck: &mut Pin<Gpio24, PushPullOutput>,
    ) -> (u8, Pin<Gpio23, PushPullOutput>) {
        let io = io.into_pull_down_input();

        let mut val = 0;

        for _ in 0..5 {
            val = (val << 1) | Self::read_bit(&io, ck);
        }
        val >>= 1;

        (val & 0b111, io.into_push_pull_output())
    }

    fn send_u8(
        io: &mut Pin<Gpio23, PushPullOutput>,
        ck: &mut Pin<Gpio24, PushPullOutput>,
        mut val: u8,
    ) {
        for _ in 0..8 {
            Self::send_bit(io, ck, val & 1);
            val >>= 1;
        }
    }

    fn read_bit(io: &Pin<Gpio23, PullDownInput>, ck: &mut Pin<Gpio24, PushPullOutput>) -> u8 {
        cortex_m::asm::delay(DELAY);
        ck.set_high().ok();
        cortex_m::asm::delay(DELAY);
        let val = defmt::unwrap!(io.is_high()) as u8;
        ck.set_low().ok();

        val
    }

    fn send_bit(
        io: &mut Pin<Gpio23, PushPullOutput>,
        ck: &mut Pin<Gpio24, PushPullOutput>,
        level: u8,
    ) {
        if level == 0 {
            io.set_low().ok();
        } else {
            io.set_high().ok();
        }
        cortex_m::asm::delay(DELAY);
        ck.set_high().ok();
        cortex_m::asm::delay(DELAY);
        ck.set_low().ok();
    }
}
