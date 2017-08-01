
int delay_time = 10 * 1000;
int wait_time = 10  * 1000;
int digpin = 2;
bool start = true;

void setup() {
  pinMode(2, OUTPUT);
}

void loop() {
  if (start) {
    delay(delay_time);
    digitalWrite(digpin, HIGH);
    digitalWrite(13, HIGH);
  }
  start = false;
  delay(wait_time);
  digitalWrite(digpin, LOW);
  digitalWrite(13, LOW);

}
