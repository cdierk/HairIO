# EEPP
Shared Repository for Expressive Tresses and Programmable Plaits

# Hardware details
- Microcontroller: Arduino Nano.
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

# TinyLily port notes
The code writes directly to the [timers](https://playground.arduino.cc/Main/TimerPWMCheatsheet) on port B. 
The TinyLily only exposes Port D and Port C. [Datasheet](https://cdn.shopify.com/s/files/1/1125/2198/files/ASM2101_Rev3.pdf?1845274497776763656). 
Pins 0 and 1 of Port D are used for Serial comms, and using them can mess up uploading/programming.
[Arduino port manipulation](https://playground.arduino.cc/Learning/PortManipulation)
[More notes on timers](http://forum.arduino.cc/index.php?topic=43581.0)

## Programming TinyLily
1. Install correct FTDI drivers for your system by [going here.](http://www.ftdichip.com/Drivers/VCP.htm)
2. Open Arduino IDE, and use the following settings:
- Board to Arduino Pro or Pro Mini
- ATMega 328 (3.3V, 8MHz)
3. Upload!

# Resources
[Instructables](http://www.instructables.com/id/Touche-for-Arduino-Advanced-touch-sensing/)

# Datasheets
[4052 mux from TI](http://www.ti.com/lit/ds/symlink/cd4051b.pdf)

# demo
One w/ everything: bluetooth, multiple braids
One that talks over bluetooth
