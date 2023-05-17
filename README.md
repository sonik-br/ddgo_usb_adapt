# ddgo_usb_adapt
Densha De Go USB adapter using a Pi Pico RP2040

I only have the PlayStation2 DDGO Type2 controller and I wanted to play the amazing english translation of the PC game DDGO Final.

So this adapter was born.

The adapter is very simple to make.
Just requires soldering an usb connector to a Pico.

Pull Requests are welcome.

## Features

Input:
- PS2 "Type2" controller `TCPP-20009`

Output:
- PC One handle controller `DGC-255`
- PC Two handle controller `DGOC-44U`
- Classic controller (for use on PS1, Saturn and N64 emulators)

Output support is planned for:
- Switch One handle controller `ZKNS-001`

A PS4, PS5 or Xbox controller can be used to help with input mapping on emulators.

## Building
Check the wiring guidance [here](https://github.com/sekigon-gonnoc/Pico-PIO-USB/discussions/7).

I'm not using any resistor and I don't think it's needed.

Pins can be changed.
Define the `D+` pin on sketch. `D-` will be `D+` + 1.

Check the arduino sketch file for required libs.

## Credits
Densha De Go [controller docs](https://marcriera.github.io/ddgo-controller-docs) from by marcriera

[Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) by sekigon-gonnoc

[tusb_xinput](https://github.com/Ryzee119/tusb_xinput) by Ryzee119

## Disclaimer

Code and wiring directions are provided to you 'as is' and without any warranties. Use at your own risk.