# RPi Pico Probe

This firmware implements an CMSIS-DAP v1 and v2 compatible probe on the RPi Pico.

It's currently using a bit-banged driver, so expect max speeds around 2 MHz.

## Current pinout

You can change this to whatever you want in `setup.rs`.

| Pin    | Description |
| ------ | ----------- |
| GPIO13 | nRESET      |
| GPIO14 | SWDIO       |
| GPIO15 | SWCLK       |

## Building

You can build the project and generate a `.uf2` file as follows:

```console
# Install elf2uf2-rs
cargo install elf2uf2-rs

# Build the ELF without logging
DEFMT_LOG=off cargo build --release --bin pico-probe 

# Generate .uf2 file
elf2uf2-rs target/thumbv6m-none-eabi/release/pico-probe pico-probe
```

Start the Pico in bootloader mode and drop the `pico-probe.uf2` file to it, done! 

## TODO

- [ ] Add support for SWO (Manchester encoding via PIO)
- [ ] Add support for VCP (it enumerates now, but ignores all data)
- [ ] Move SWD impl to PIO
- [ ] Document the `dap-rs` traits and helpers
- [ ] Document the firmware

