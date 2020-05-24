void stop()
{
  digitalWrite(L_FORW, 0);
  digitalWrite(L_BACK, 0);
  digitalWrite(R_FORW, 0);
  digitalWrite(R_BACK, 0);
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}
