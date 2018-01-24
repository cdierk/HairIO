# EEPP
Shared Repository for Expressive Tresses and Programmable Plaits

# Arduino code
[mux_4052](https://github.com/cdierk/EEPP/tree/master/4052_mux_test_code) - newer version of mux that only requires two pins to select between 4 inputs. Uses [Disney's Touche that was ported to Arduino.](http://www.instructables.com/id/Touche-for-Arduino-Advanced-touch-sensing/)

# Arduino pinout notes
## For mux_4052 code
  pinMode(9, OUTPUT);       //-Signal generator pin  // digital PWM
  pinMode(8, OUTPUT);       //-Sync (test) pin       // digital
  pinMode(LED, OUTPUT);
  pinMode(muxApin, OUTPUT);                          // pin 6, digital PWM
  pinMode(muxBpin, OUTPUT);                          // pin 5, digital PWM
  pinMode(braidOutput, OUTPUT);                      // pin 2, digital
  // Also uses Analog 0 later.
  
  Total: 5 digital pins, 1 analog pin.

# Programming TinyLily
1. Install correct FTDI drivers for your system by [going here.](http://www.ftdichip.com/Drivers/VCP.htm)
2. Open Arduino IDE, and use the following settings:
- Board to Arduino Pro or Pro Mini
- ATMega 328 (3.3V, 8MHz)
3. Upload!

# Resources
[Instructables](http://www.instructables.com/id/Touche-for-Arduino-Advanced-touch-sensing/)

# Datasheets
[4052 mux from TI](http://www.ti.com/lit/ds/symlink/cd4051b.pdf)
