void eDrive2(int dir) {
  if (dir > 0) {
    drive2(1);
  } else if (dir < 0) {
    drive2(-1);
  } else {
    drive2(0);
  }

  error1 = FL.encoderTick - FR.encoderTick;
  error2 = RL.encoderTick - RR.encoderTick;

  FLPower += error1 * 100 / kp;
  RLPower += error2 * 100 / kp;
    
  if (FLPower < 1) {
    FLPower = 0;
  } else if (FLPower >= 255) {
    FLPower = 255;
  }
  
  if (RLPower < 1) {
    RLPower = 0;
  } else if (FRPower >= 255) {
    RLPower = 255;
  }

   /*
  Serial.print("Error:");
  Serial.print(error1);
  Serial.print(" LRPM:");
  Serial.print(FL.encoderTick);
  Serial.print(" LSPD:");
  Serial.print(FLPower);
  Serial.print(" RRPM:");
  Serial.print(FR.encoderTick);
  Serial.print(" RSPD:");
  Serial.println(rightPower);
 */

/*
  Serial.print("Error:");
  Serial.print(error2);
  Serial.print(" LRPM:");
  Serial.print(RL.encoderTick);
  Serial.print(" LSPD:");
  Serial.print(RLPower);
  Serial.print(" RRPM:");
  Serial.print(RR.encoderTick);
  Serial.print(" RSPD:");
  Serial.println(RLPower);
*/

  FL.encoderTick = 0;
  FR.encoderTick = 0;
  
  RL.encoderTick = 0;
  RR.encoderTick = 0;
  //leftEncoder = 0;
 // rightEncoder = 0;

  // delay(50);
  /*
    Serial.print(" ");
    Serial.print(leftPower);
    Serial.print("leftPower RPM");
    Serial.print(rpmA);

    Serial.print("\t");
    Serial.print(rightPower);
    Serial.print("rightPower RPM");
    Serial.println(rpmB);
  */
}

void eDrive(int dir) {
  if (dir > 0) {
    drive(1);
  } else if (dir < 0) {
    drive(-1);
  } else {
    drive(0);
  }

  error = FL.encoderTick - FR.encoderTick;

  leftPower += error / kp;
  if (leftPower < 1) {
    leftPower = 0;
  } else if (leftPower >= 255) {
    leftPower = 255;
  }
  Serial.print("Error:");
  Serial.print(error);
  Serial.print(" LRPM:");
  Serial.print(FL.encoderTick);
  Serial.print(" LSPD:");
  Serial.print(leftPower);
  Serial.print(" RRPM:");
  Serial.print(FR.encoderTick);
  Serial.print(" RSPD:");
  Serial.println(rightPower);

  FL.encoderTick = 0;
  FR.encoderTick = 0;
  leftEncoder = 0;
  rightEncoder = 0;

  // delay(50);
  /*
    Serial.print(" ");
    Serial.print(leftPower);
    Serial.print("leftPower RPM");
    Serial.print(rpmA);

    Serial.print("\t");
    Serial.print(rightPower);
    Serial.print("rightPower RPM");
    Serial.println(rpmB);
  */
}

//Encoder Drive Functions
void driveStraight(int dir) {
  if (dir > 0) {
    drive(1);
  } else if (dir < 0) {
    drive(-1);
  } else {
    drive(0);
  }

  error = leftEncoder - rightEncoder;

  leftPower += error / kp;
  if (leftPower < 1) {
    leftPower = 0;
  } else if (leftPower >= 255) {
    leftPower = 255;
  }
  Serial.print("Error:");
  Serial.print(error);
  Serial.print(" LRPM:");
  Serial.print(leftEncoder);
  Serial.print(" LSPD:");
  Serial.print(leftPower);
  Serial.print(" RRPM:");
  Serial.print(rightEncoder);
  Serial.print(" RSPD:");
  Serial.println(rightPower);

  leftEncoder = 0;
  rightEncoder = 0;

  delay(50);

  Serial.print(" ");
  Serial.print(leftPower);
  Serial.print("leftPower RPM");
  Serial.print(rpmA);

  Serial.print("\t");
  Serial.print(rightPower);
  Serial.print("rightPower RPM");
  Serial.println(rpmB);
}


void drive2 (int dir) {
  if (dir > 0) {
    analogWrite(lenB, FLPower);
    digitalWrite(lin1, HIGH);
    digitalWrite(lin2, LOW);
    analogWrite(lenA, RLPower);
    digitalWrite(lin3, LOW);
    digitalWrite(lin4, HIGH);

    analogWrite(renB, FRPower);
    digitalWrite(rin1, LOW);
    digitalWrite(rin2, HIGH);
    analogWrite(renA, RRPower);
    digitalWrite(rin3, HIGH);
    digitalWrite(rin4, LOW);
  } else if (dir < 0) {
    analogWrite(lenB, FLPower);
    digitalWrite(lin1, LOW);
    digitalWrite(lin2, HIGH);
    analogWrite(lenA, RLPower);
    digitalWrite(lin3, HIGH);
    digitalWrite(lin4, LOW);

    analogWrite(renB, FRPower);
    digitalWrite(rin1, HIGH);
    digitalWrite(rin2, LOW);
    analogWrite(renA, RRPower);
    digitalWrite(rin3, LOW);
    digitalWrite(rin4, HIGH);
  } else if (dir == 0) {  //Stop
    analogWrite(lenB, 0);
    analogWrite(lenA, 0);
    analogWrite(renB, 0);
    analogWrite(renA, 0);
  }
}

//Basic Drive Functions(Drive and Turn Based On Time)
void drive (int dir) {
  if (dir > 0) {
    analogWrite(lenB, leftPower);
    digitalWrite(lin1, HIGH);
    digitalWrite(lin2, LOW);
    analogWrite(lenA, leftPower);
    digitalWrite(lin3, LOW);
    digitalWrite(lin4, HIGH);

    analogWrite(renB, rightPower);
    digitalWrite(rin1, LOW);
    digitalWrite(rin2, HIGH);
    analogWrite(renA, rightPower);
    digitalWrite(rin3, HIGH);
    digitalWrite(rin4, LOW);
  } else if (dir < 0) {
    analogWrite(lenB, leftPower);
    digitalWrite(lin1, LOW);
    digitalWrite(lin2, HIGH);
    analogWrite(lenA, leftPower);
    digitalWrite(lin3, HIGH);
    digitalWrite(lin4, LOW);

    analogWrite(renB, rightPower);
    digitalWrite(rin1, HIGH);
    digitalWrite(rin2, LOW);
    analogWrite(renA, rightPower);
    digitalWrite(rin3, LOW);
    digitalWrite(rin4, HIGH);
  } else if (dir == 0) {  //Stop
    analogWrite(lenB, 0);
    analogWrite(lenA, 0);
    analogWrite(renB, 0);
    analogWrite(renA, 0);
  }
}

void turn(int dir) {
  if (dir > 0) {
    analogWrite(lenB, Power);
    digitalWrite(lin1, HIGH);
    digitalWrite(lin2, LOW);
    analogWrite(lenA, Power);
    digitalWrite(lin3, LOW);
    digitalWrite(lin4, HIGH);

    analogWrite(renB, Power);
    digitalWrite(rin1, HIGH);
    digitalWrite(rin2, LOW);
    analogWrite(renA, Power);
    digitalWrite(rin3, LOW);
    digitalWrite(rin4, HIGH);
  } else if (dir < 0) {
    analogWrite(lenB, Power);
    digitalWrite(lin1, LOW);
    digitalWrite(lin2, HIGH);
    analogWrite(lenA, Power);
    digitalWrite(lin3, HIGH);
    digitalWrite(lin4, LOW);

    analogWrite(renB, Power);
    digitalWrite(rin1, LOW);
    digitalWrite(rin2, HIGH);
    analogWrite(renA, Power);
    digitalWrite(rin3, HIGH);
    digitalWrite(rin4, LOW);
  }
}
