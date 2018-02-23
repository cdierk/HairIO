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
#define driveSignalPin 2 // drive circuit signal pin
#define voltageMonitorPin A4 //analog pin for monitoring the drive battery

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
 * pins specific to a particular braid
 * are stored in a braidx object
 */
struct braidx {
  int muxPin;
  int thermPin;
  int capTouchPin;
};

/**
 * hardware has two braid channels
 * TODO double check the pins
 */
struct braidx braid0 = {muxApin, A0, 5};
struct braidx braid1 = {muxBpin, A1, 6};
struct braidx currentBraid = braid0;

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

  // mux, right now this is trivial, we are only dealing with one braid
  digitalWrite(muxApin, 1);
  digitalWrite(muxBpin, 0);
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
  }
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
void thermistorThrottle() {
    int average = readThermistor();
    if (average >= TEMPTHRESHOLD) {
        digitalWrite(driveSignalPin, LOW);
        driving = false;
    }
}
void loop()
{
  for (unsigned int d = 0; d < N; d++)
  {
    int v = analogRead(currentBraid.capTouchPin);  //-Read response signal
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


  //this would send the data to processing
  //PlottArray(1,freq,results);

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
    //Serial.println(received);
    if (received.equals("!B10;")){
      digitalWrite(driveSignalPin,HIGH);
      driving = true;
    } else if (received.equals("!B219")){
      digitalWrite(driveSignalPin,LOW);
      driving = false;
    }
    //read out the temperature and write to serial monitor
    if (driving) {
      readThermistor();
    }
  }


  TOG(PORTB, 0);           //-Toggle pin 8 after each sweep (good for scope)
}


