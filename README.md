# Rusty Probe

This firmware implements an CMSIS-DAP v1 and v2 compatible probe.

## Building

You can build the project and generate a `.uf2` file as follows:

```console
# Install elf2uf2-rs and flip-link (you only need to do this once)
cargo install elf2uf2-rs flip-link

# Build the ELF without logging
DEFMT_LOG=off cargo build --release --bin app

# Generate .uf2 file
elf2uf2-rs target/thumbv6m-none-eabi/release/app app
```

Start the RP2040 in bootloader mode and drop the `app.uf2` file to it, done! 

## Running with `defmt` logs without debugger

If you don't have a debugger/programmer to program or debug your rusty-probe with, you can still run the probe and see the defmt logs it prints.

To do so, start the probe in the USB bootloader by powering it on while holding the button pressed. Mount the resulting block device.

Then, perform the following steps:

```console
# Install elf2uf2-rs, flip-link, and defmt-print (you only need to do this once)
cargo install elf2uf2-rs flip-link defmt-print

# Build the binary with the desired level of logging, and run
# it using `xtask`
DEFMT_LOG=debug cargo rrb-usb app

# If `xtask` says it found multiple probes, choose a serial
# number from its output:
DEFMT_LOG=debug cargo rrb-usb app --serial $SERIAL
```

You can now repeat the above command, which will automatically restart and flash your Rusty Probe. Note that this does require that the block device is mounted each time.

You may have to specify a different --vendor and --product depending on the identifiers used by your probe, and on non-x86 linux platforms, you must update the runner configuration with the correct target (see [`.cargo/config.toml`](.cargo/config.toml#L15)).

## Virtual serial port

rusty-probe-firmware supports a CDC-ACM virtual serial port on the USB side, and forwards the data back and forth to pins 20 (TX from the `rusty-probe`) and 21 (RX into the `rusty-probe`) on UART1 by default.  It uses the line coding (baud, data bits, parity, stop bits) from the host port configuration (on Linux, `stty -F /dev/ttyACMx 115200 cs8 -parenb -cstopb` for 115200/8/N/1, for example).

Pins 20 and 21 can be connected to pins 13 (target RX) and 14 (target TX) of a STDC14 connector, and from there to a microcontroller serial port RX and TX pins.  Note that regardless of the connector and routing, TX on the `rusty-probe` must connect to RX on the target device, and vice versa.

## Feature TODO list

- [ ] Add support for JTAG (only SWD works right now)
- [ ] Add support for SWO

