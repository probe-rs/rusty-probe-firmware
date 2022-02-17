# RPi Pico Probe

This firmware implements an CMSIS-DAP v1 and v2 compatible probe on the RPi Pico.

It's currently using a bit-banged driver, so expect max speeds around 2 MHz.


## Current pinout

| Pin    | Description |
| ------ | ----------- |
| GPIO13 | nRESET      |
| GPIO14 | SWDIO       |
| GPIO15 | SWCLK       |

