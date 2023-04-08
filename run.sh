#!/bin/sh

set -e

# To use a different serial port name, set the `LOG_SERIAL` environment
# variable to the filename that you expect your serial port to have.
serial=${LOG_SERIAL-"/dev/ttyACM0"}
file=${1-"./target/thumbv6m-none-eabi/release/app"}

check_exists() {
    if [ ! -f "$(which $1 2> /dev/null)" ]; then
        echo "Could not find $1. Please install it."
        exit 1
    fi
}

check_exists elf2uf2-rs
check_exists defmt-print
check_exists stty

./restart.sh "$serial"
elf2uf2-rs -d "$file"

echo "Waiting for & attaching to serial"

# In the absence of inotifywait, we busy-wait for
# the serial device to appear.
# Maybe we can replace this with some inotifywait magic
while [ ! -e "$serial" ]; do
    :
done

sudo stty -F "$serial" raw 115200 && sudo cat "$serial" | defmt-print --show-skipped-frames -e "$file"