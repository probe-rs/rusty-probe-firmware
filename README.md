# Rusty Probe

This firmware implements an CMSIS-DAP v1 and v2 compatible probe.

## Building

You can build the project and generate a `.uf2` file as follows:

```console
# Install elf2uf2-rs
cargo install elf2uf2-rs

# Build the ELF without logging
DEFMT_LOG=off cargo build --release --bin app

# Generate .uf2 file
elf2uf2-rs target/thumbv6m-none-eabi/release/app app
```

Start the Pico in bootloader mode and drop the `app.uf2` file to it, done! 

## Running with `defmt` logs without debugger

If you don't have a debugger/programmer to program or debug your rusty-probe with, you can still run the probe and see the defmt logs it prints.

To do so, start the probe with the bootloader enabled by powering it on while holding
the button pressed. Mount the resulting block device.

Then, perform the following steps:

```console
# Install elf2uf2-rs and defmt-print (you only need to do this once)
cargo install elf2uf2-rs defmt-print

# Build the ELF file with the desired level of logging and the
# correct defmt transport.
DEFMT_LOG=debug cargo build --release --bin app  \
        --no-default-features                    \
        --features defmt-bbq

# Flash the ELF, attach to the serial port, and print the defmt logs
elf2uf2-rs -s -d target/thumbv6m-none-eabi/release/app \
    | defmt-print -e ./target/thumbv6m-none-eabi/release/app
```

## TODO

- [x] Move SWD impl to PIO
- [ ] Add support for SWO (Manchester encoding or UART via PIO)
- [ ] Add support for VCP (it enumerates now, but ignores all data)
- [ ] Add the automatic polling of RTT buffers
- [ ] Document the `dap-rs` traits and helpers
- [ ] Document the firmware

