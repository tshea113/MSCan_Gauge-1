MSCan_Gauge
===========

This project uses a [Teensy 3.2 microcontroller](https://www.pjrc.com/teensy/index.html) to interface with a [Megasquirt 3 open source standalone
engine controller](http://www.msextra.com/) via CAN bus. Data is received from the ECU and shown on a display that fits in a standard 52mm gauge pod.

Due to Teensy 3.2 availability, work is in progress to get a board working with [Teensy 4.0](https://www.pjrc.com/store/teensy40.html) which is both more powerful and more available.

## Hardware used
* [Teensy 3.2 microcontroller](https://www.pjrc.com/store/teensy32.html)
* [Adafruit 128x64 monochrome OLED display](http://www.adafruit.com/products/938) connected via SPI.
* [Adafruit 16 LED Neopixel ring](http://www.adafruit.com/products/1463) (optional)
* [Adafruit rotary encoder](http://www.adafruit.com/products/377)

## Libraries required
* [Metro](https://www.pjrc.com/teensy/arduino_libraries/Metro.zip) - event scheduling
* [Time](https://www.pjrc.com/teensy/arduino_libraries/Time.zip) - onboard clock
* [FlexCAN](https://github.com/collin80/FlexCAN_Library) - CAN bus
* [SSD1306](https://github.com/adafruit/Adafruit_SSD1306) - OLED display
* [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) - required by SSD1306
* [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO) - I2C and SPI library
* [FastLED](https://github.com/FastLED/FastLED) - LED ring
* [EncoderTool](https://www.pjrc.com/teensy/arduino_libraries/Encoder.zip) - rotary encoder
* [Bounce2](https://github.com/thomasfredericks/Bounce2) - button debouncing

## Component BOM
Passives are 1206 package unless otherwise noted.
* R1 - 10K
* C1 - 10uF
* IC1 - SN65HVD230 (or similar) CAN transceiver
* VR1 - TSR 1-2450 DC/DC Converter (500mA may work but 1A is recommended)
* 2.54mm spacing pin headers - standard length (5 pins for OLED screen, 4 pins for encoder, and 3 for optional led ring)
* 8 way Molex Micro-fit+ connectors (Cable mount male and PCB mount female) and associated pins
* 5, 4, and 3 position mating headers or connectors for OLED screen, encoder, and led ring connections respectively
