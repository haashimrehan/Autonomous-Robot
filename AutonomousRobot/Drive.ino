//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  else pos_left--;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
  else pos_right++;
}

void stop()
{
  leftJrk.stopMotor();
  rightJrk.stopMotor();
}

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
