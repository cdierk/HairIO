
int delay_time = 10 * 1000;
int wait_time = 10  * 1000;
int digpin = 7;
bool start = true;

void setup() {
  pinMode(digpin, OUTPUT);
}

void loop() {
  if (start) {
    delay(delay_time);
  
   for (int i = 0; i < 60; i++) {
    onoff();
   }
  
  
  }
  start = false;
  digitalWrite(digpin, LOW);
  digitalWrite(13, LOW);

}


void onoff() {
    delay(100);
    digitalWrite(digpin, HIGH);
    digitalWrite(13, HIGH);
    delay(400);
    digitalWrite(digpin, LOW);
    digitalWrite(13, LOW);
}

