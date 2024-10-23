#![no_std]

use panic_probe as _;

use defmt_brtt as _;

use rtic_monotonics::Monotonic;
use setup::Mono;

pub mod dap;
pub mod device_signature;
pub mod leds;
pub mod pio;
pub mod setup;
pub mod systick_delay;
pub mod usb;

defmt::timestamp! {"{=u64:us}", {
    Mono::now().duration_since_epoch().to_micros()
}
}
