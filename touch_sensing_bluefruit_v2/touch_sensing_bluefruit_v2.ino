/**
 * Demo code for bluetooth enabled PCB
 * ***********************************
 * Uses Bluefruit app on phone to receive captouch signals and toggle driving
 * Monitors temperature via thermistor; includes drive shutoff if temperature threshold exceeded
 * Monitors battery voltages; warning light and drive shutoff if voltages below threshold
 */

//****************************************************************************************
// Adapted from https://github.com/Illutron/AdvancedTouchSensing
// Moves the gesture analysis from Processing to run entirely on Arduino, with fixed thresholds
// July 21 2017
//
//Added Bluetooth code adapted from Adafruit nRF51822 Bluefruit source code
//****************************************************************************************
//****************************************************************************************
// Illutron take on Disney style capacitive touch sensor using only passives and Arduino
// Dzl 2012
//****************************************************************************************


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

#include <Arduino.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"


/**
 * capacitive touch definitions
 */
#define SET(x,y) (x |=(1<<y))        //-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))           // |
#define CHK(x,y) (x & (1<<y))               // |
#define TOG(x,y) (x^=(1<<y))                //-+
#define N 160  //How many frequencies

/**
 * hardware pins
 */
#define muxApin 6
#define muxBpin 5
#define CAPTOUCHPIN A3
#define driveSignalPin 2 // drive circuit signal pin
#define voltageMonitorPin A4 //analog pin for monitoring the drive battery
#define onboardLED 13

/**
 * thermistor specific constants
 */
#define THERMISTORNOMINAL 10000 //resistance at 25 degrees C
#define TEMPERATURENOMINAL 25 // temperature for nominal resistance
#define NUMSAMPLES 5 //how many samples to take before averaging the temperature
#define BCOEFFICIENT 3950 //beta coefficient of the thermistory
#define SERIESRESISTOR 10000 //value of 'other' resistor
#define TEMPTHRESHOLD 28.0

/**
 * voltage thresholds for batteries
 */
#define SAFE_DRIVE_VOLTAGE_THRESHOLD 3.5
#define SAFE_CONTROL_VOLTAGE_THRESHOLD 5

/**
 * pins specific to a particular braid
 * are stored in a braidx object
 * the channel on the mux is set by the state of the
 * A and B select channels; the relevant states are saved 
 * in the braid object.
 * ch0 = A0B0
 * ch1 = A1B0
 */
struct braidx {
  int muxSelectA;
  int muxSelectB;
  int thermPin;
};

/**
 * hardware has two braid channels
 */ 
struct braidx braid0 = {0, 0, A0}; //left
struct braidx braid1 = {1, 0, A1}; //right
struct braidx currentBraid = braid1;

/**
 * Bluetooth setup
 * Create the bluefruit object in software serial
 */
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


/**
 * Gesture sensing and processing variables
 */
//Gesture sensing variables
int results[N];            //-Filtered result buffer
int freq[N];            //-Filtered result buffer
int sizeOfArray = N;

//Gesture processing variables
float gesturePoints[2][2];
int gestureDist[2];
String names[2] = {"nothing", "touch"};

char curGesture = 0;
char lastGesture = 0;

//so that we don't send "braid touched" while drive circuit is running
bool driving = false; 

/****************************************
 * End of variable set up
 ****************************************/

void setup()
{ 
  Serial.begin(9600);
  Serial.println("touch_sensing_bluefruit_v2 edited February 22th, 2018");
  
  setGestureThresholds();

  TCCR1A = 0b10000010;      //-Set up frequency generator
  TCCR1B = 0b00011001;      //-+
  ICR1 = 110;
  OCR1A = 55;

  pinMode(9, OUTPUT);       //-Signal generator pin
  pinMode(8, OUTPUT);       //-Sync (test) pin
  pinMode(muxApin, OUTPUT);
  pinMode(muxBpin, OUTPUT);
  pinMode(driveSignalPin, OUTPUT);
  pinMode(onboardLED, OUTPUT);

  // initialize results array to all zeros
  memset(results,0,sizeof(results));

  ble.begin();

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  ble.info();
  
  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // Set module to DATA mode
  //Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  //set mux pins according to braid's channel select parameters
  digitalWrite(muxApin, currentBraid.muxSelectA);
  digitalWrite(muxBpin, currentBraid.muxSelectB);
}


void setGestureThresholds() {
  gesturePoints[0][0] = 34;
  gesturePoints[0][1] = 390;

  gesturePoints[1][0] = 50;
  gesturePoints[1][1] = 300;
}


void processGesture() {
  if ((curGesture == 1) & (driving == false)){
    String s = F("braid touched                                       ");
    uint8_t sendbuffer[20];
    s.getBytes(sendbuffer, 20);
    char sendbuffersize = min(20, s.length());

    ble.write(sendbuffer, sendbuffersize);
    Serial.println(s);
  }
}

void sendBLEMessage(String msg) {
  uint8_t sendbuffer[20];
  msg.getBytes(sendbuffer, 20);
  char sendbuffersize = min(20, msg.length());

  ble.write(sendbuffer, sendbuffersize);
}

//adapted from http://forum.arduino.cc/index.php?topic=41999.0
int getMaxFromArray(int* array, int size) {
  int max = array[0];
  for (int i = 1; i < size; i++) {
    if (max < array[i]) {
      max = array[i];
    }
  }
  return max;
}

//assumes no negative values for time or voltage
int dist(int x1, int y1, int x2, int y2) {

  int xmax = max(x1, x2);
  int ymax = max(y1, y2);

  float w;
  if (x1 > x2) {
    w = x1 - x2;
  } else {
    w = x2 - x1;
  }
  float h;
  if (y1 > y2) {
    h = y1 - y2;
  } else {
    h = y2 - y1;
  }
  
  int intVal = (int) sqrt(h*h + w*w);
  return intVal;
}

void analyzeInput(int timeArr[], int voltageArr[]) {

  /* ====================================================================
    Gesture compare
    ====================================================================  */
  int currentMax = 0;
  int currentMaxValue = -1;
  for (int i = 0; i < 2; i++)
  {
    //calculate individual dist
    gestureDist[i] = dist(getMaxFromArray(timeArr, N), getMaxFromArray(voltageArr, N), gesturePoints[i][0], gesturePoints[i][1]);
    if (gestureDist[i] < currentMaxValue || i == 0)
    {
      currentMax = i;
      currentMaxValue =  gestureDist[i];
    }
  }
  
  int type = currentMax;
  lastGesture = curGesture;
  curGesture = type;
}


// read temp and write to serial monitor
// return temp
int readThermistor() {
    uint8_t i;
    float average;
    uint16_t samples[NUMSAMPLES];

    //take N samples in a row, with a slight delay
  
    for (i = 0; i < NUMSAMPLES; i++) {
      samples[i] = analogRead(currentBraid.thermPin); //doesn't matter right now, but we need this to correspond to the right braid
      delay(10);
    }
  
    // average all the samples out
    average = 0;
    for (i = 0; i < NUMSAMPLES; i++) {
      average += samples[i];
    }
    average /= NUMSAMPLES;
  
    //convert these values to resistance
    average = 1023 / average - 1;
    average = SERIESRESISTOR / average;
  
    //convert these values to temperature
    average = average / THERMISTORNOMINAL;
    average = log(average);
    average /= BCOEFFICIENT;
    average += 1.0 / (TEMPERATURENOMINAL + 273.15);
    average = 1.0 / average;
    average -= 273.15;
  
    //average is now the temperature of the thermistor
    Serial.println(average);
  
    return average;
}

//If the temperature is above the threshold,
//turn off driving
void thermistorThrottle(float temp) {
    if (temp >= TEMPTHRESHOLD) {
        turnOffDrive();
    }
}

void capacitiveSweep() {
  for (unsigned int d = 0; d < N; d++)
  {
    int v = analogRead(CAPTOUCHPIN);  //-Read response signal
    CLR(TCCR1B, 0);         //-Stop generator
    TCNT1 = 0;              //-Reload new frequency
    ICR1 = d;               // |
    OCR1A = d / 2;          //-+
    SET(TCCR1B, 0);         //-Restart generator

    int testVal = results[d] * 0.5 + v*0.5;
    results[d] = testVal;
    //results[d] = results[d] * 0.5 + (float)(v) * 0.5; //Filter results
    freq[d] = d;

    //   plot(v,0);              //-Display
    //   plot(results[d],1);
    // delayMicroseconds(1);
  }
}

//https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


/*
 * Monitor the state of the batteries.
 * Light the onboard LED if the battery is low
 * TODO: need a way to distinguish which one is low
 */
void monitorBatteries() {
  int driveSensorValue = analogRead(voltageMonitorPin);
  float driveVoltage= driveSensorValue * (3.7 / 1023.0);

  Serial.println(driveVoltage);
//  if (driveVoltage < SAFE_DRIVE_VOLTAGE_THRESHOLD) {
//      turnOffDrive();
//      digitalWrite(onboardLED, HIGH);
//  }
  long controlVoltageMillivolts = readVcc();
  float controlVoltage = controlVoltageMillivolts/1000.0;
  Serial.println(controlVoltage);
//  if (controlVoltage < SAFE_CONTROL_VOLTAGE_THRESHOLD) {
//      digitalWrite(onboardLED, HIGH);
//  }
}

void turnOnDrive() {
   digitalWrite(driveSignalPin, HIGH);
   driving = true;
   Serial.println("Driving...");
   sendBLEMessage("Driving.............................................");
}

void turnOffDrive() {
   digitalWrite(driveSignalPin, LOW);
   driving = false;
   Serial.println("Not driving...");
   sendBLEMessage("Not driving.........................................");
}

void loop()
{
  //update the captouch state
  capacitiveSweep();
  
  //this would send the data to processing
//  PlottArray(1,freq,results);

  // instead of sending to processing, do the work here
  analyzeInput(freq, results);
  processGesture();

  String received = "";
  
  // Check to see if received toggle from Phone
  while ( ble.available() )
  {
    int c = ble.read();
    received = received + (char) c;
    //Serial.println((char)c);
    Serial.println(received);
    if (received.equals("!B10;")){      //turn the hair drive on
        turnOnDrive();
    } else if (received.equals("!B219")){ //turn the hair drive off
        turnOffDrive();
    }
    
    //read out the temperature and write to serial monitor
  }
//  if (driving) {
//    float temp = readThermistor();
////    thermistorThrottle(temp); //turn it off if temp is too high
//  }

//  monitorBatteries();

  TOG(PORTB, 0);           //-Toggle pin 8 after each sweep (good for scope)
}


