use core::ptr::addr_of;
use cortex_m::interrupt;
use rp2040_hal::pac;
use rp2040_hal::rom_data;

pub fn device_id_hex() -> &'static str {
    static mut DEVICE_ID_STR: [u8; 22] = [0; 22];

    unsafe {
        interrupt::free(|_| {
            if DEVICE_ID_STR.as_ptr().read_volatile() == 0 {
                let hex = b"0123456789abcdef";
                for (i, b) in read_uid().iter().chain(read_jedec().iter()).enumerate() {
                    let lo = b & 0xf;
                    let hi = (b >> 4) & 0xf;
                    DEVICE_ID_STR[i * 2] = hex[hi as usize];
                    DEVICE_ID_STR[i * 2 + 1] = hex[lo as usize];
                }
            }
        });
        core::str::from_utf8_unchecked(&*addr_of!(DEVICE_ID_STR))
    }
}

#[inline(always)]
#[link_section = ".data.ram_func"]
unsafe fn set_cs(level: bool) {
    (&*pac::IO_QSPI::ptr())
        .gpio_qspiss()
        .gpio_ctrl()
        .modify(|_, w| {
            if level {
                w.outover().high()
            } else {
                w.outover().low()
            }
        });
}

#[link_section = ".data.ram_func"]
#[inline(never)]
unsafe fn do_flash_cmd(txrxbuf: &mut [u8]) {
    // Load important addresses to the stack
    let connect_internal_flash = rom_data::connect_internal_flash;
    let flash_exit_xip = rom_data::flash_exit_xip;
    let flash_flush_cache = rom_data::flash_flush_cache;

    let mut boot2: core::mem::MaybeUninit<[u8; 256]> = core::mem::MaybeUninit::uninit();

    let xip_base = 0x10000000 as *const u32;
    rom_data::memcpy(boot2.as_mut_ptr() as _, xip_base as _, 256);

    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);

    connect_internal_flash();
    flash_exit_xip();

    set_cs(false);

    let ssi = &*pac::XIP_SSI::ptr();

    for b in txrxbuf {
        while !ssi.sr().read().tfnf().bit_is_set() {}
        ssi.dr0().write(|w| w.dr().bits(*b as _));

        while !ssi.sr().read().rfne().bit_is_set() {}
        *b = ssi.dr0().read().dr().bits() as _;
    }

    set_cs(true);

    flash_flush_cache();

    let ptr = (boot2.as_mut_ptr() as *const u8).add(1) as *const ();
    let start: extern "C" fn() = core::mem::transmute(ptr);
    start();
}

fn read_uid() -> [u8; 8] {
    const FLASH_RUID_CMD: u8 = 0x4b;
    const FLASH_RUID_DUMMY_BYTES: usize = 4;
    const FLASH_RUID_DATA_BYTES: usize = 8;
    const FLASH_RUID_TOTAL_BYTES: usize = 1 + FLASH_RUID_DUMMY_BYTES + FLASH_RUID_DATA_BYTES;

    let mut buf = [0; FLASH_RUID_TOTAL_BYTES];
    buf[0] = FLASH_RUID_CMD;

    unsafe {
        do_flash_cmd(&mut buf);
    }

    buf[FLASH_RUID_DUMMY_BYTES + 1..].try_into().unwrap()
}

fn read_jedec() -> [u8; 3] {
    const FLASH_JEDEC_CMD: u8 = 0x9f;
    const FLASH_JEDEC_TOTAL_BYTES: usize = 4;

    let mut buf = [0; FLASH_JEDEC_TOTAL_BYTES];
    buf[0] = FLASH_JEDEC_CMD;

    unsafe {
        do_flash_cmd(&mut buf);
    }

    buf[1..].try_into().unwrap()
}
