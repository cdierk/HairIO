# HairIO: Human Hair as interactive Material
Shared Repository for HairIO: Human Hair as interactive Material

# Hardware details
- Microcontroller: Arduino Nano, ordered from AliExpress. Requires [these drivers.](https://github.com/MPParsley/ch340g-ch34g-ch34x-mac-os-x-driver/)
- 

# Circuit diagram for SFCS from Disney's Touche project, as shared by Illutron
// Illutron take on Disney style capacitive touch sensor using only passives and Arduino
// Dzl 2012


```
//                              10n
// PIN 9 --[10k]-+-----10mH---+--||-- OBJECT
//               |            |
//              3.3k          |
//               |            V 1N4148 diode
//              GND           |
//                            |
//Analog 0 ---+------+--------+
//            |      |
//          100pf   1MOmhm
//            |      |
//           GND    GND
```

# Arduino code
- [mux_4052](https://github.com/cdierk/EEPP/tree/master/4052_mux_test_code) - newer version of mux that only requires two pins to select between 4 inputs. Uses [Disney's Touche that was ported to Arduino.](http://www.instructables.com/id/Touche-for-Arduino-Advanced-touch-sensing/)
- [touch_sensing_bluefruit_v2](https://github.com/cdierk/EEPP/tree/master/touch_sensing_bluefruit_v2) - Currently believe this is what's in the bluetooth-enabled demo box w/ brown hair that can actuate and communicate over bluetooth.

# Arduino pinout notes
## For mux_4052 code
- pinMode(9, OUTPUT);       //-Signal generator pin  // digital PWM
- pinMode(8, OUTPUT);       //-Sync (test) pin       // digital
- pinMode(LED, OUTPUT);
- pinMode(muxApin, OUTPUT);                          // pin 6, digital PWM
- pinMode(muxBpin, OUTPUT);                          // pin 5, digital PWM
- pinMode(braidOutput, OUTPUT);                      // pin 2, digital
- // Also uses Analog 0 later.
  
  Total: 5 digital pins, 1 analog pin.
  
## For touch_sensing_bluefruit_v2 code
// cap-sense pin
- pinMode(9, OUTPUT);       //-Signal generator pin
- pinMode(8, OUTPUT);       //-Sync (test) pin
- pinMode(drive, OUTPUT);

 //bluetooth:
- BLUEFRUIT_UART_MODE_PIN,
- BLUEFRUIT_UART_CTS_PIN, 
- BLUEFRUIT_UART_RTS_PIN

# Resources
[Instructable for Disney Touche's SFCS](http://www.instructables.com/id/Touche-for-Arduino-Advanced-touch-sensing/)

# Datasheets and Circuit Info
- [4052 mux from TI](http://www.ti.com/lit/ds/symlink/cd4051b.pdf)
- [Arduino Nano CAD files](https://forum.arduino.cc/index.php?topic=373897.0)
- [SPDT Slide switch datasheet from DigiKey](https://media.digikey.com/pdf/Data%20Sheets/C&K/SS-12D07-VG_GA_PA.pdf)
  - For Eagle, use a 1x3-pin header from Pinhead since they are also 2mm apart.

# demo
- Multiple braids w/ BLE
- Single braid w/ BLE
- 

# Breakout board notes

- EPD breakout board (green):
- Compatible with: 40+pin e-ink display (connector is zig-zag).
- Goals: Recreate it on the breadboard, then simplify if possible, then create an SMD version.

- Teensy e-paper adapter board (red): https://hackaday.io/project/13327-teensy-e-paper-adapter-board
- Compatible with: 20+-pin e-ink displays (connector is straight).
- Goals: Get it working (library code freezes and we don't know why), recreate and build our own.



