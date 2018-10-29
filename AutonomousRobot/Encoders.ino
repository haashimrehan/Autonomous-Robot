void updateEncoderA()
{
  encoderValueA++;
  leftEncoder++;
}

void updateEncoderB()
{
  encoderValueB++;
  rightEncoder++;
}

void updateRPM(bool Print) {
  // Update RPM value on every second
  currentMillisA = millis();
  if (currentMillisA - previousMillisA > interval) {
    previousMillisA = currentMillisA;

    rpmA = (float)(encoderValueA * 60 / ENCODEROUTPUT);

    if (Print) {
      Serial.print(encoderValueA);
      Serial.print(" ");
      Serial.print(rpmA);
      Serial.println(" RPMA");
    }
    encoderValueA = 0;

    currentMillisB = millis();
    if (currentMillisB - previousMillisB > interval) {
      previousMillisB = currentMillisB;

      rpmB = (float)(encoderValueB * 60 / ENCODEROUTPUT);

      if (Print) {
        Serial.print(encoderValueB);
        Serial.print(" ");
        Serial.print(rpmB);
        Serial.println(" RPMB");
      }
      encoderValueB = 0;
    }
  }
}
