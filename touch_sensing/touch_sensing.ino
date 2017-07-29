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



#define SET(x,y) (x |=(1<<y))        //-Bit set/clear macros
#define CLR(x,y) (x &= (~(1<<y)))           // |
#define CHK(x,y) (x & (1<<y))               // |
#define TOG(x,y) (x^=(1<<y))                //-+

#define LED 13 // LED for serial-less debugging.

#define N 160  //How many frequencies

//Gesture sensing variables
float results[N];            //-Filtered result buffer
float freq[N];            //-Filtered result buffer
int sizeOfArray = N;

//Gesture processing variables
float gesturePoints[2][2];
float gestureDist[2];
String names[2] = {"nothing", "touch"};

char curGesture = 0;
char lastGesture = 0;

void setGestureThresholds() {
  gesturePoints[0][0] = 34;
  gesturePoints[0][1] = 540;

  gesturePoints[1][0] = 50;
  gesturePoints[1][1] = 400;
}


void setup()
{

  setGestureThresholds();

  TCCR1A = 0b10000010;      //-Set up frequency generator
  TCCR1B = 0b00011001;      //-+
  ICR1 = 110;
  OCR1A = 55;

  pinMode(9, OUTPUT);       //-Signal generator pin
  pinMode(8, OUTPUT);       //-Sync (test) pin
  pinMode(LED, OUTPUT);

  // initialize results array to all zeros
  memset(results,0,sizeof(results));
}




void processGesture() {
  if (curGesture == 1) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
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


  TOG(PORTB, 0);           //-Toggle pin 8 after each sweep (good for scope)
}


