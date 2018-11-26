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

  if (!stopped && fcm < pingStopDistance){
    //Serial.println("STOPPED");
    //Serial.println(fcm);
    drive(0);
    Power = 0;
    rightPower = 0;
    leftPower = 0;
    FLPower = 0;
    FRPower = 0;
    RLPower = 0;
    RRPower = 0;
    stopped = true;
    stillThere = 0;
    delay(2000);
  }

  if (stopped && fcm > pingStopDistance) {
    //Serial.println("STARTED");
    //Serial.println(fcm);
    stopped = false;
    Power = maxPower;
    rightPower = maxPower;
    leftPower = maxPower;
    FLPower = maxPower;
    FRPower = maxPower;
    RLPower = maxPower;
    RRPower = maxPower;
  }
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
