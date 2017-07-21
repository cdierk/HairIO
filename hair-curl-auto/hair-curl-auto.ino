//const int hairPin = 3;
const int highPin = 3;
const int lowPin = 4;
//const int ledPin = 13;

void setup() {
  //pinMode(hairPin, OUTPUT);
  //pinMode(ledPin, OUTPUT);
  pinMode(highPin, OUTPUT);
  pinMode(lowPin, OUTPUT);
}

void loop() {
  digitalWrite(highPin, HIGH);
  digitalWrite(lowPin, LOW);

/*
  
  digitalWrite(ledPin, HIGH);
  digitalWrite(hairPin, HIGH);
  delay(5000);
  digitalWrite(hairPin, LOW);
  digitalWrite(ledPin, LOW);
  delay(3000);
  */
}
