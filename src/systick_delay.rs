use cortex_m::peripheral::{syst::SystClkSource, SYST};

pub struct Delay {
    systick: SYST,
    ticks_per_us: u32,
}

impl Delay {
    pub fn new(mut systick: SYST, cpu_frequency: u32) -> Self {
        // Set clock source to processor clock
        systick.set_clock_source(SystClkSource::Core);

        // Set reload and current values
        systick.set_reload(0xffffff);
        systick.clear_current();

        // Enable the counter
        systick.enable_counter();

        Delay {
            systick,
            ticks_per_us: (cpu_frequency + 500_000) / 1_000_000,
        }
    }

    pub fn delay_us(&self, mut us: u32) {
        while us > 0x1fff {
            let ticks = (us & 0x1fff) * self.ticks_per_us;
            self.delay_ticks(ticks as u32);
            us -= us & 0x1fff;
        }
    }

    pub fn delay_ticks(&self, mut ticks: u32) {
        let mut last = self.get_current();
        loop {
            let now = self.get_current();
            let delta = last.wrapping_sub(now) & 0xffffff;

            if delta >= ticks {
                break;
            } else {
                ticks -= delta;
                last = now;
            }
        }
    }

    pub fn delay_ticks_from_last(&self, mut ticks: u32, mut last: u32) -> u32 {
        loop {
            let now = self.get_current();
            let delta = last.wrapping_sub(now) & 0xffffff;

            if delta >= ticks {
                break now;
            } else {
                ticks -= delta;
                last = now;
            }
        }
    }

    #[inline(always)]
    pub fn get_current(&self) -> u32 {
        self.systick.cvr.read()
    }
}

unsafe impl Sync for Delay {}
