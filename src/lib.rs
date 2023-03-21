#![no_std]

use panic_probe as _;

#[cfg(feature = "defmt-rtt")]
use defmt_rtt as _;

#[cfg(feature = "defmt-bbq")]
use defmt_bbq as _;

use rp2040_hal::timer::Instant;
use rtic_monotonics::{rp2040::Timer, Monotonic};

pub mod dap;
pub mod device_signature;
pub mod pio;
pub mod setup;
pub mod systick_delay;
pub mod usb;

defmt::timestamp! {"{=u64:us}", {
    Timer::now().checked_duration_since(Instant::from_ticks(0)).map(|i| i.ticks()).unwrap_or(0)
}
}
