void drive (int dir) {
  if (dir > 0) {
    analogWrite(pwmB, Power);
    digitalWrite(dir1B, HIGH);
    digitalWrite(dir2B, LOW);
    analogWrite(pwmA, Power);
    digitalWrite(dir1A, LOW);
    digitalWrite(dir2A, HIGH);
  } else if (dir < 0) {
    analogWrite(pwmB, Power);
    digitalWrite(dir1B, LOW);
    digitalWrite(dir2B, HIGH);
    analogWrite(pwmA, Power);
    digitalWrite(dir1A, HIGH);
    digitalWrite(dir2A, LOW);
  } else if (dir == 0) {  //Stop
    analogWrite(dir1A, 0);
    analogWrite(dir2A, 0);
    analogWrite(dir1B, 0);
    analogWrite(dir2B, 0);
  }
}

void turn(int dir) {
  if (dir > 0) {
    analogWrite(pwmB, Power);
    digitalWrite(dir1B, HIGH);
    digitalWrite(dir2B, LOW);
    analogWrite(pwmA, Power);
    digitalWrite(dir1A, HIGH);
    digitalWrite(dir2A, LOW);

  } else if (dir < 0) {
    analogWrite(pwmB, Power);
    digitalWrite(dir1B, LOW);
    digitalWrite(dir2B, HIGH);
    analogWrite(pwmA, Power);
    digitalWrite(dir1A, LOW);
    digitalWrite(dir2A, HIGH);

  }
}
