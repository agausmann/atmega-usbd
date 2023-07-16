# atmega-usbd

`usb_device` support for ATmega microcontrollers.

NOTE: This is an experimental library, and certainly not production-ready. Expect bugs,
and please help out by reporting them if you find any! If you need absolute reliability,
consider a more mature framework like [LUFA].

## MCU support

Currently supports:

- ATmega16u4/32u4

Untested but probably close to being supported:

- ATmega8u2/16u2/32u2 - USB peripheral is very similar to m16u4.

## Examples

The [`arduino_keyboard` example](examples/arduino_keyboard.rs) is a "Hello
World" example that can be run on an Arduino Leonardo. Usage is documented in
the example file.

[LUFA]: http://www.fourwalledcubicle.com/LUFA.php
[rust-avr-usb]: https://github.com/agausmann/rust-avr-usb
