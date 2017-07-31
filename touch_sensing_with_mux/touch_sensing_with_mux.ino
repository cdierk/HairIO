//****************************************************************************************
// Adapted from https://github.com/Illutron/AdvancedTouchSensing
// Moves the gesture analysis from Processing to run entirely on Arduino, with fixed thresholds
// July 21 2017
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

//EDIT THIS ONE YAY!
float tempThreshhold = 28.0;

#define SET(x,y) (x |=(1<<y))        //-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))           // |
#define CHK(x,y) (x & (1<<y))               // |
#define TOG(x,y) (x^=(1<<y))                //-+

#define LED 13 // LED for serial-less debugging.

#define N 160  //How many frequencies

//mux
#define muxApin 6
#define muxBpin 5
#define muxCpin 4
int previousMillis = 0;
int switchInterval = 100;

unsigned int currentMillis = 0;

#define braid0 2    //pin for braid 0 output
#define braid1 3    //pin for braid 1 output
#define braid2 7    //pin for braid 2 output

//thermistor specific stuff
#define therm0 A1   //pin for braid 0 thermistor
#define therm1 A2   //pin for braid 1 thermistor
#define therm2 A3   //pin for braid 2 thermistor
//resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temperature for nominal resistance
#define TEMPERATURENOMINAL 25
//how many samples to take before averaging the temperature
#define NUMSAMPLES 5
//beta coefficient of the thermistor
#define BCOEFFICIENT 3950
//value of 'other' resistor
#define SERIESRESISTOR 10000

int currentBraid = braid0;
int currentTherm = therm0;

uint16_t samples[NUMSAMPLES];

bool driving = false; //true if currently driving one of the braids

//Gesture sensing variables
float results[N];            //-Filtered result buffer
float freq[N];            //-Filtered result buffer
int sizeOfArray = N;

//Gesture processing variables
float gesturePoints[2][2];
float gestureDist[2];
String names[2] = {"nothing", "touch"};

int curGesture = 0;
int lastGesture = 0;

void setGestureThresholds() {
  gesturePoints[0][0] = 34;
  gesturePoints[0][1] = 540;

  gesturePoints[1][0] = 50;
  gesturePoints[1][1] = 400;
}


void setup()
{

  Serial.begin(115200);

  setGestureThresholds();

  TCCR1A = 0b10000010;      //-Set up frequency generator
  TCCR1B = 0b00011001;      //-+
  ICR1 = 110;
  OCR1A = 55;

  pinMode(9, OUTPUT);       //-Signal generator pin
  pinMode(8, OUTPUT);       //-Sync (test) pin
  pinMode(LED, OUTPUT);
  pinMode(muxApin, OUTPUT);
  pinMode(muxBpin, OUTPUT);
  pinMode(muxCpin, OUTPUT);
  pinMode(braid0, OUTPUT);
  pinMode(braid1, OUTPUT);
  pinMode(braid2, OUTPUT);

  // initialize results array to all zeros
  memset(results,0,sizeof(results)); 
}




void processGesture() {
  //input found on current braid
  if ((curGesture == 1) && (currentMillis > 1000)){
    digitalWrite(currentBraid, HIGH);
    driving = true;
  } 
}

//adapted from http://forum.arduino.cc/index.php?topic=41999.0
float getMaxFromArray(float* array, int size) {
  int max = array[0];
  for (int i = 1; i < size; i++) {
    if (max < array[i]) {
      max = array[i];
    }
  }
  return max;
}

//assumes no negative values for time or voltage
float dist(float x1, float y1, float x2, float y2) {
  float xmax = max(x1, x2);
  float ymax = max(y1, y2);

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

  return sqrt(h * h + w * w);
}

void analyzeInput(float timeArr[], float voltageArr[]) {

  /* ====================================================================
    Gesture compare
    ====================================================================  */
  int currentMax = 0;
  float currentMaxValue = -1;
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

void loop()
{
  unsigned int d;

  int counter = 0;
  for (unsigned int d = 0; d < N; d++)
  {
    int v = analogRead(0);  //-Read response signal
    CLR(TCCR1B, 0);         //-Stop generator
    TCNT1 = 0;              //-Reload new frequency
    ICR1 = d;               // |
    OCR1A = d / 2;          //-+
    SET(TCCR1B, 0);         //-Restart generator

    results[d] = results[d] * 0.5 + (float)(v) * 0.5; //Filter results

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

    //mux -- cycles through braids looking for input
    currentMillis = millis();
    
    //doesn't cycle if powering one of the braids
    if((currentMillis - previousMillis > switchInterval) && (driving == false)){
      // save the last time you switched 
      previousMillis = currentMillis;   

      if (currentBraid == braid0){               //mux pin0 is attached to same braid as Arduino pin2
        digitalWrite(muxApin, 0);
        digitalWrite(muxBpin, 0);
        digitalWrite(muxCpin, 0);
        currentBraid = braid1;
      } else if (currentBraid == braid1){        //mux pin1 is attached to same braid as Arduino pin3
        digitalWrite(muxApin, 1);
        digitalWrite(muxBpin, 0);
        digitalWrite(muxCpin, 0);
        currentBraid = braid2;
      } else {        //mux pin2 is attached to same braid as Arduino pin7
        digitalWrite(muxApin, 0);
        digitalWrite(muxBpin, 1);
        digitalWrite(muxCpin, 0);
        currentBraid = braid0;
      }
    }
    //only care about thermistor values if we're currently driving
      if (driving == true) {
        uint8_t i;
        float average;

        //take N samples in a row, with a slight delay
        if (currentBraid == braid0) currentTherm = therm0;
        else if (currentBraid == braid1) currentTherm = therm1;
        else if (currentBraid == braid2) currentTherm = therm2;
        
        for (i = 0; i < NUMSAMPLES; i++) {
          samples[i] = analogRead(currentTherm); //doesn't matter right now, but we need this to correspond to the right braid
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

        if (average >= tempThreshhold){
          digitalWrite(currentBraid, LOW);
          driving = false;
        }
      }
   

  TOG(PORTB, 0);           //-Toggle pin 8 after each sweep (good for scope)
}


