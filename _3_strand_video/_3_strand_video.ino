
int delay_time = 10 * 1000;
int wait_time = 60  * 1000;
int slow_time = 1 * 1000;
bool start = true;

void setup() {
  pinMode(2, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop() {
  if (start) {
    delay(delay_time);
    digitalWrite(2, HIGH);
    delay(wait_time);
    digitalWrite(2, LOW);

    delay(slow_time);

    digitalWrite(10, HIGH);
    delay(wait_time);
    digitalWrite(10, LOW);

    delay(slow_time);

    digitalWrite(12, HIGH);
    delay(wait_time);
    digitalWrite(12, LOW);

  
  }
  start = false;
  digitalWrite(2, LOW);
  digitalWrite(10, LOW);
  digitalWrite(12, LOW);
}
