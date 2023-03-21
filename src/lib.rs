#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};
use panic_probe as _;

#[cfg(feature = "defmt-rtt")]
use defmt_rtt as _;

#[cfg(feature = "defmt-bbq")]
use defmt_bbq as _;

pub mod dap;
pub mod device_signature;
pub mod pio;
pub mod setup;
pub mod systick_delay;
pub mod usb;

defmt::timestamp! {"{=u64}", {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n as u64
}
}
