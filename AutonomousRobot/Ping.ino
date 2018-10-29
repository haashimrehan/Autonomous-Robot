void pingSense(bool Print) {
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);

  pingFilter.in(cm);
  fcm = pingFilter.out();

  if (Print) {
    Serial.print(fcm);
    Serial.print(" cm");
    Serial.println();
  }
  pingSafety();
  delay(1);
}

void pingSafety() {
  
    if (!stopped && cm < pingStopDistance) {
      drive(0);
      Power = 0;
      rightPower = 0;
      leftPower = 0;
      stopped = true;
      delay(2000);
    }
    if (stopped && cm > pingStopDistance){
      stopped = false;
      Power = maxPower;
      rightPower = maxPower;
      leftPower = maxPower;
    }
  /*
    tempR = rightPower;
      tempL = leftPower;
      tempSpd = Power;

      if (cm < pingStopDistance && !stopped) {
      drive(0);
      Power = 0;
      rightPower = 0;
      leftPower = 0;
      delay(1000);
      stopped = true;
      }
      if (stopped && cm > pingStopDistance) {
      Power = tempSpd;
      rightPower = tempR;
      leftPower = tempL;
      }*/
}

long microsecondsToCentimeters(long microseconds)
{
  int value = microseconds / 29 / 2;
  if (value > 350) {
    return 10;
  } else {
    return value;
  }
}
