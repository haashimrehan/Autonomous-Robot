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
