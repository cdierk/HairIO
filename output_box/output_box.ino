
int hair_pins[5] = {13, 10, 8, 4, 2};
int switch_pin = 2;

//initialize in off state
int active_hair = 5;
int state = LOW;

void setup() {
  // put the hair pins into output mode 
  for (int i = 0; i < 5; i++) {
    pinMode(hair_pins[i], OUTPUT);
  }
  // put the switch pins into input mode
  pinMode(switch_pin, INPUT);

  // initialize serial for debugging
  Serial.begin(115200);
}

void loop() {

// switch the active hair
  if (state == LOW && digitalRead(switch_pin) == HIGH) {
    //debounce  
    delay(20);
    
    if (digitalRead(switch_pin) == HIGH) {
      active_hair = (active_hair + 1) % 6; 
      state = HIGH;
      //write the output
      write_to_pins();

          Serial.print("Active_hair:");
          Serial.print(active_hair);
          Serial.print("\n");
    }  
  }

  if (digitalRead(switch_pin) == LOW) {
    state = LOW;
  }

}


void write_to_pins() {
  //  if we've reached end of cycle, turn it all off
  if (active_hair == 5) {
      for (int i = 0; i < 5; i++) {
        digitalWrite(hair_pins[i], LOW);
      }
  } 
//  otherwise turn the active hair on, and everything else off
  else {  
    for (int i = 0; i < 5; i++) {
      if (i == active_hair) {
        digitalWrite(hair_pins[i], HIGH);
      } else {
        digitalWrite(hair_pins[i], LOW);
      } 
    } 
  }
}

