MSCan_Gauge
===========

This project uses a Teensy 3.1 microcontroller (https://www.pjrc.com/teensy/index.html) to interface with a Megasquirt 3 open source standalone
engine controller (http://www.msextra.com/) via CAN bus. Data is both received from the ECU and shown on a display and sent from attached devices back to the ECU.

## Hardware used
* The Teensy has an onboard CAN controller, however it needs a transceiver (Ebay, etc).
* [Adafruit 128x64 monochrome OLED display](http://www.adafruit.com/products/938) connected via SPI.
* [Adafruit 3 axis digital accelerometer](http://www.adafruit.com/products/1231) connected via i2c.
* [Adafruit Ultimate GPS breakout](http://www.adafruit.com/products/746) connected via serial.
* [Adafruit 16 LED Neopixel ring](http://www.adafruit.com/products/1463)
* [Adafruit rotary encoder](http://www.adafruit.com/products/377)

## Libraries required
* [Metro](https://www.pjrc.com/teensy/arduino_libraries/Metro.zip) - event scheduling
* [Time](https://www.pjrc.com/teensy/arduino_libraries/Time.zip) - onboard clock
* [FlexCAN](https://github.com/teachop/FlexCAN_Library) - CAN bus
* [Adafruit ADXL345](https://github.com/adafruit/Adafruit_ADXL345/archive/master.zip) - accelerometer
* [Adafruit sensor lib](https://github.com/adafruit/Adafruit_Sensor) - required by accelerometer lib
* [SSD1306](https://www.pjrc.com/teensy/arduino_libraries/Adafruit_SSD1306.zip) - OLED display
* [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) - required by SSD1306
* [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) - GPS
* [FastLED](https://github.com/FastLED/FastLED) - LED ring
* [Encoder](https://www.pjrc.com/teensy/arduino_libraries/Encoder.zip) - rotary encoder

## Component BOM
Passives are 1206 package unless otherwise noted.
* R1 - 10K
* R2 - 120 ohm
* R3 - 10K (only required for headlight sense)
* R4 - 4K (only required for headlight sense)
* C1 - 10uF
* IC1 - SN65HVD230 (or similar) CAN transceiver
* VR1 - Recom R-78 5V switching regulator. 500mA may work but 1A is recommended. LM7805 analog regulator is not recommended due to there being not enough room for a heatsink
* 2.54mm spacing pin headers - various lengths. 15mm for GPS and OLED (OLED may need trimmed). Standard length for everything else. 8 position right angle female for OLED
* 2.54mm spacing 8 position screw terminal for outside connections
* 3 and 4 position mating headers or connectors for encoder and led ring connections
