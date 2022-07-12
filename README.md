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

The `arduino_keyboard` example is a "Hello World" example that can be run on an
Arduino Leonardo, following these steps:

1. (Optional) Connect a pushbutton switch to the D2 pin of the Leonardo, and connect the other pin
  of the switch to GND.

2. Connect the Leonardo to the computer with a USB cable.

3. Make sure [Ravedude] is installed.

4. "Run" the example - Ravedude is configured as a runner for this project:

  ```
  cargo run --release --example arduino_keyboard
  ```
  
5. Open Notepad (or whatever editor or text input of your choosing).

6. Press the button (or if you are not using one, short D2 to GND with a jumper).
You should see it type "Hello World"

[LUFA]: http://www.fourwalledcubicle.com/LUFA.php
[rust-avr-usb]: https://github.com/agausmann/rust-avr-usb
[Ravedude]: https://github.com/Rahix/avr-hal/tree/main/ravedude
