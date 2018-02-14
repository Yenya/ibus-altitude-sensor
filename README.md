# I-Bus Altitude Sensor for FlySky Telemetry

## Requirements

* FlySky transmitter with AFHDS-2A telemetry. Preferably FS-i6 with
  modified [firmware from qba667](https://github.com/qba667/FlySkyI6).
* Receiver with I-Bus sensor input, such as FS-iA6B. No firmware modification
  required.
* Atmel AVR-based board. I used SparkFun
  [ProMicro](https://www.sparkfun.com/products/12640) clone,
  and this is what the firmware expects. It is about the lightest but powerful
  ATmega-based board.
* Pressure sensor with I2C on a break-out board. I used BMP-180 on a GY-68
  board bought at eBay. There are more precise sensors, such as BMP-280
  (TODO: try these as well). Current firmware expects BMP-180.
* 390K and 15K resistors for voltage divider for reading the battery
  voltage (optional).
* some wires and a 3-pin servo female connector for I-Bus.
* AVR toolchain (avr-gcc, avr-libc, avrdude, make, ...).

## Connection

Here is how to connect things together:

![wiring](wiring.png)

And here is my (rather messy) real-world example in Sky Surfer 1400:

![Wiring in Sky Surfer 1400](wiring-photo.jpg)

## Firmware

The firmware is based on two libraries:

* AVR [BMP085 library](https://davidegironi.blogspot.cz/2012/10/avr-atmega-bmp085-pressure-sensor.html) by Davide Gironi
* [I2C/TWI master library](http://homepage.hispeed.ch/peterfleury/doxygen/avr-gcc-libraries/group__pfleury__ic2master.html) by Peter Fleury

The I-Bus code is in the main file [ibus-sensor.c]. It provides the following
sensors:

* Temperature sensor from BMP 185
* Altitude relative to the power-on value
* Absolute altitude (reported as GPS-Altitude sensor)
* Maximum relative altitude reached since power on
* Climb rate
* External voltage (can be used for battery voltage alarm; optional)

It also provides visual feedback about the main loop (on-board Tx LED)
and the I-Bus communication (on-board Rx LED).

## License

This project, as well as its libraries, is licensed under the terms
of GNU General Public License, version 2 (only). See the file [COPYING].

## Author

Written by [Jan "Yenya" Kasprzak](https://www.fi.muni.cz/~kas/).
