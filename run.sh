#!/bin/bash

serial=${LOG_SERIAL-"/dev/ttyACM0"}
file=${1-"./target/thumbv6m-none-eabi/release/app"}

./restart.sh "$serial"
elf2uf2-rs -d "$file"

sleep 2

sudo stty -F "$serial" raw 115200 && sudo cat "$serial" | defmt-print --show-skipped-frames -e "$file"