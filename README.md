MSCan_Gauge
===========

This project uses a [Teensy 3.2 or 4.0 microcontroller](https://www.pjrc.com/teensy/index.html) to interface with a [Megasquirt open source standalone
engine controller](http://www.msextra.com/) via CAN bus. Data is received from the ECU and shown on a display that fits in a standard 52mm gauge pod.

Due to Teensy 3.2 availability, the project has been ported to work with [Teensy 4.0](https://www.pjrc.com/store/teensy40.html) which is both more powerful and more available. Highly suggest using that hardware over the Teensy 3.2.

## Hardware used
* [Teensy 4.0 microcontroller](https://www.pjrc.com/store/teensy40.html)
* [Adafruit 128x64 monochrome 1.3" OLED display](http://www.adafruit.com/products/938) connected via SPI.
* [Adafruit rotary encoder](http://www.adafruit.com/products/377)

Unfortunately this display from Adafruit is pretty expensive compared to some of the cheaper no-name brands.
The gauge housing is fitted to work with this specific display model, but, although untested, any one of the SSD1306 driver controlled displays should work (as long as the screen is 128x64 pixels).

The rotary encoder is P/N: PEC11-4215F-S24 (or any substituteable encoder). If you are already ordering from Adafruit, it's best to just grab theirs.

## Libraries required
* [Metro](https://github.com/thomasfredericks/Metro-Arduino-Wiring) - event scheduling
* [Time](https://github.com/PaulStoffregen/Time) - onboard clock
* [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4) - CAN bus
* [SSD1306](https://github.com/adafruit/Adafruit_SSD1306) - OLED display
* [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) - required by SSD1306
* [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO) - I2C and SPI library (required by SSD1306)
* [EncoderTool](https://github.com/luni64/EncoderTool) - rotary encoder
* [Bounce2](https://github.com/thomasfredericks/Bounce2) - button debouncing

## Component BOM
Passives are 1206 package unless otherwise noted.
* R1 - 10K
* C1 - 10uF
* IC1 - SN65HVD230 (or similar) CAN transceiver
* VR1 - TSR 1-2450 DC/DC Converter (500mA may work but 1A is recommended)
* 2.54mm spacing male pin headers - standard length (5 male pins for OLED screen and 4 male pins for encoder)
* 2.54mm spacing female socket headers - standard length (2 banks of 14 female pins for the teensy)
* 8 way Molex Micro-fit+ connectors (Cable mount male and PCB mount female) and associated pins
* 5 and 4 position mating headers or connectors for OLED screen and encoder connections respectively

## Building the device

There are a lot of individual pieces that come together in this project, so it is best to test things as much as you can each step of the way. The gauge should come on when plugged into USB while external power is disconnected.
Before setting up CAN in your car, the gauge should still come on when the car is powered on and display a message about waiting for data. When you set up the CAN connection, you should start to see data come in on the screen.

### PCB
Board gerber files are stored under Board files. Teensy 3.2 and Teensy 4.0 are *NOT* pin compatible, so choose the right PCB for your Teensy. Then encoder files are just for the rotary encoder which will be mounted remotely. I used OSHPark to build my boards, but there are plenty of PCB fabs out there to use.

The component BOM matches the labels on the PCB. You should be able to solder everything using a fine tip soldering iron. The CAN tranceiver is a little tricky, but doable.

### Housing
Highly recommended to print these out of ABS/ASA. I used PLA as a prototype and it *WILL* deform. Print the main housing using supports, but the PCB tray and the face should be able to just print flat on the print bed.

You will need the following fasteners:
* 8 M2 nuts
* 4 M2 bolts 6mm length
* 4 M2 bolts 10mm length

Fasten the PCB to the tray by placing 4 of the M2 nuts in the slots on the bottom. Use the shorter 6mm bolts to secure the PCB. The tray should slide right into the housing with the Molex connector coming out the back.
The tray may be tight at first, but sliding it in and out a few times should clearance it enough to be smooth.

Place 4 of the M2 Nuts into the back of the holes for the display mounts. Use a bolt to pull the nuts into their slots. Once they are in, they should be secure enough that they do not fall out.

To connect the display, you will need 5 female-to-female connectors. I used some pre-made 3 inch wires that have female header pins on the ends. Tuck the wires back into the gauge and place the screen into the slot.
Secure the display using the 10mm bolts. The gauge face should slot right into the housing.

### Car wiring
The wiring in the car should be wired up to the male Micro-fit+ connector. You will need to supply:
* 12V power
  * Preferably something that isn't hot while cranking.
  * This will probably be the same supply you use for other gauges like wideband.
* Ground
  * Nothing specific here, this isn't a sensor ground or anything.
* ENCG
  * This is the ground for the encoder. It should wire up to the corresponding pin on the encoder PCB.
* ENCB
  * This is the button pin for the encoder. It should wire up to the corresponding pin on the encoder PCB.
* ENC1
  * This is one of the direction pins for the encoder. It should wire up to the corresponding pin on the encoder PCB.
* ENC2
  * This is the other direction pin for the encoder. It should wire up to the corresponding pin on the encoder PCB.
* CANL
  * CAN low signal. This should wire up to the MegaSquirt.
* CANH
  * CAN high signal. This should wire up to the MegaSquirt.

CANL and CANH wires are a differential signal and should be twisted together coming from the ECU to whatever device you are connecting them to (in this case the gauge).

The CAN bus requires a 120Ohm termination at the end of the bus. I did not include this on the gauge, since other devices may already have this termination.
If this gauge is your only CAN device, you can use a Deutsh connector with this termination on the end:
* DTM06-2S-P006
  * This is the 120Ohm termination
* DTM04-2P
  * This is the female receptical.
  * You will also need pins and the little plastic bracket. Read more on Deutsch connectors to see what parts you need.

The Molex Micro-fit+ pins are very small and delicate. They do not require any special crimping tools, and I was able to use a generic pin crimping tool without issues. The pins should slot into the male housing and you will feel a click when they are secure.
If you aren't gentle with them, the wings on the end that secure the pins will get bent. To fix this, pull out the pin, carefully bend the wings back, and try again.

Connect the Molex connector to the gauge, and the gauge should come on when the ignition is in the ON position.

### Tunerstudio
You will need to configure CAN Realtime Data Broadcasting for the data you are interested in.
* Make sure the baseID is set to 1520.
* Data rate for each item will depend on how fast you expect the data to change. Things like coolant temp and MAT are going to be very slow while things like RPM and MAP are going to be faster.
* Be sure the master enable for CAN is set under CAN Paramteters
* Also enable CAN Broadcasting under CAN Broadcasting

## Future Development

There probably won't be any development on this project outside of bug fixes to make sure this stays in a good state.
I'm working on a new project that will allow me to fix some of the headaches with this design:
* Updating the diplay to use the GC9A01A circular LCD display.
  * LCDs are much easier to see in the sunlight and this display has RGB colors.
* Changing the project to use PlatformIO
  * This will allow the project to store libraries in GitHub. You will be able to sync the repo and build it directly!
  * I can use GitHub Actions to automate builds in the repo.
  * Lots of nice tools inside of PlatformIO IDE.
* Moving away from the Arduino style coding to something more C++ based.
  * Partially enabled by PlatformIO, since it just builds using CMake.
  * This project grew in scope enough where the lack of OOP was making it difficult to manage data & states.

New development will be under the teensy-CAN-gauge project: https://github.com/tshea113/teensy-CAN-gauge/
