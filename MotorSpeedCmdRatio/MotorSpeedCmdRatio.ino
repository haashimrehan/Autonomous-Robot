// Uses PLX-DAQ to log motor Speed vs motor cmd to find slope and constant

#include <JrkG2.h>

JrkG2I2C leftJrk(12);
JrkG2I2C rightJrk(11);

#define LOOPTIME                      100

const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor
const int PIN_ENCOD_B_MOTOR_LEFT = 4;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;              //A channel for encoder of right motor
const int PIN_ENCOD_B_MOTOR_RIGHT = 5;              //B channel for encoder of right motor

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

const double radius = 0.04;                   //Wheel radius, in m
const double wheelbase = 0.35;               //Wheelbase, in m
const double encoder_cpr = 330;                //Encoder ticks or counts per rotation

//const double speed_to_pwm_ratio = 0.000429;     //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).

const double left_speed_to_cmd_ratio = 0.000429;     //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double left_min_speed_cmd = -0.983;           //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

const double right_speed_to_cmd_ratio = 0.000765;     //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double right_min_speed_cmd = -1.57;           //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

unsigned long lastMilli = 0;

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s
double leftRPM = 0;

void setup() {

  // open serial connection
  Serial.begin(9600);
  Wire.begin();
  //Serial.println("CLEARDATA"); // clears sheet starting at row 2
  Serial.println("CLEARSHEET"); // clears sheet starting at row 1

  // define 5 columns named "Date", "Time", "Timer" and "millis"
  Serial.println("LABEL,Date,Time,Timer,millis,leftSpeed,rightSpeed,Command");

  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(0, encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT);
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT);
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(1, encoderRightMotor, RISING);
}

long cmd = 2048;
int changeVal = 50;

long elapsedTime = 0;

void loop() {
  leftJrk.setTarget(cmd);
  rightJrk.setTarget(cmd);

  if (cmd == 2048) {
    long e1 = millis();
    while (millis() - e1 < 7000) {
    }
  }

  if (cmd > 4040) {
    changeVal = -50;
  }
  if (cmd < 60) {
    changeVal = 50;
  }
  if (millis() - elapsedTime > 2000)
  {
    cmd += changeVal;
    elapsedTime = millis();
  }

  if ((millis() - lastMilli) >= LOOPTIME)
  { // enter timed loop
    lastMilli = millis();

    if (abs(pos_left) < 2) {                                                 //Avoid taking in account small disturbances
      speed_act_left = 0;
    } else {
      speed_act_left = ((pos_left / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of left wheel
    }

    if (abs(pos_right) < 2) {                                                 //Avoid taking in account small disturbances
      speed_act_right = 0;
    } else {
      speed_act_right = ((pos_right / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of right wheel
    }

    pos_left = 0;
    pos_right = 0;
  }

  if (millis() - elapsedTime > 1000 && millis() - elapsedTime < 1200) {
    Serial.print("DATA,DATE,TIME,TIMER,");
    Serial.print(millis()); Serial.print(",");
    Serial.print(speed_act_left); Serial.print(",");
    Serial.print(speed_act_right); Serial.print(",");
    Serial.println(cmd);
    Serial.print(","); Serial.println("SCROLLDATA_20");
  }

}


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
