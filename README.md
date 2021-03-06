# Enix: The Modern Nixie Clock
### A beautifully sleek product that gives new life to vintage components
#### Learn more at our website: https://www.argonprototypes.com/
This repository contains hardware, software, CAD files, and much more!

![](https://i.imgur.com/bQGN5OA.jpg)

# More info about each section:
## Electronic Hardware:
The circuit board was designed in EAGLE CAD V9.2.2. Most dimensions are in mm on this board. The Gerber files are in RS-274X and Excellon drill format. There are several external libraries used; Sparkfun, Adafruit, MacroFab, and a couple more for unique parts. There should not be any problems with opening the design files without the libraries installed.

![Bottom Board](https://i.imgur.com/zrHhSTt.png)
![Top Board](https://i.imgur.com/f9MKZg2.png)
## Software:
The majority of the software is written in VS Code with the PlatformIO extension, however the Arduino IDE works just as well. The microcontroller used is the ATMega32u4, which has an 8 bit architecture with a 16MHz clock speed, is 5V tolerant, and includes built in USB programmability. This microcontroller is found on the Arduino Micro, which can be selected from the IDE to be able to upload to the board.<br/> The libraries used are Arduino.h, TimerOne.h, RTClib.h, Adafruit_NeoPixel.h, Adafruit MCP9808.h, Wire.h, and EEPROM.h.



## Case
The outer case is a CNC milled block of Aluminum or Wood. The Aluminum cases are Vapour Blasted to give a satin look, and then Anodized in Black or Clear to add a protective coating. The Wood cases are sanded to a smooth finish, prior to coating in polyurethane or oil. <br/> To hold the electronics in the case, there is a 3D printed bottom plate which uses heat set inserts to bolt to the circuit board.

## Packaging
To safely ship the Enix, a custom cardboard box with the internal dimensions of 6x6x3" was designed and ordered through Packlane. 2 Foam CNC milled inserts tightly pack the clock and all included accessories, which includes the Enix, some stickers, a drink coaster, instruction manual, and a Mini-USB cable.<br/>
The Packlane box is shipped in a larger 8x8x5" standard cardboard box filled with packing peanuts to give added protection in shipping.
