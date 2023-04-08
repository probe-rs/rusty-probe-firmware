#!/bin/sh

set -e

file=${1-"/dev/ttyACM0"}

if [ -e "$file" ]; then
    echo "Restarting pico"

    sudo stty -F "$file" 115200 raw
    echo -ne "\xDE\xAD\xBE\xEF" | sudo tee "$file" > /dev/null

    sleep 5
else
    echo "Could not restart pico"
fi