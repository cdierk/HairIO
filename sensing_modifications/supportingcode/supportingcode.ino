void watch_thermister() {
    uint8_t i;
    float average;
  
    //take N samples in a row, with a slight delay
    if (currentBraid == braid0) currentTherm = therm0;
    else if (currentBraid == braid1) currentTherm = therm1;
    else if (currentBraid == braid2) currentTherm = therm2;
    
    for (i = 0; i < NUMSAMPLES; i++) {
      samples[i] = analogRead(currentTherm);
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
  //        Serial.println(average);
  
    if ( average <= 0.0){
      digitalWrite(currentBraid, LOW);
      driving = false;
      delay(5000);
      
    } else if (average >= tempThreshhold){
  //          Serial.println("here inside kill");
      digitalWrite(currentBraid, LOW);
      driving = false;
    }
}

