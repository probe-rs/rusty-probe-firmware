#!/bin/bash

set -e

serial=${LOG_SERIAL-"/dev/ttyACM0"}
file=${1-"./target/thumbv6m-none-eabi/release/app"}

check_exists() {
    if [ ! -f $(which "$1") ]; then
        echo "Could not find $1. Please install it. (probably \`cargo install \"$1\"\`)"
        exit 1
    fi
}

check_exists elf2uf2-rs
check_exists defmt-print

./restart.sh "$serial"
elf2uf2-rs -d "$file"

sleep 2

echo "Attaching to serial"

sudo stty -F "$serial" raw 115200 && sudo cat "$serial" | defmt-print --show-skipped-frames -e "$file"