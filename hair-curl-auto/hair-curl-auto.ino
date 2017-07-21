const int hairPin = 3;
const int ledPin = 13;

void setup() {
  pinMode(hairPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);
  digitalWrite(hairPin, HIGH);
  delay(5000);
  digitalWrite(hairPin, LOW);
  digitalWrite(ledPin, LOW);
  delay(3000);
}
