#include <JrkG2.h>
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <math.h>

//initializing all the variables
#define LOOPTIME                      100 //115     //Looptime in millisecond (10hz)
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

double speed_cmd_left2 = 0;

const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor
const int PIN_ENCOD_B_MOTOR_LEFT = 4;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;              //A channel for encoder of right motor
const int PIN_ENCOD_B_MOTOR_RIGHT = 5;              //B channel for encoder of right motor

unsigned long lastMilli = 0;

const double radius = 0.04;                   //Wheel radius, in m
const double wheelbase = 0.35;               //Wheelbase, in m
const double encoder_cpr = 330;                //Encoder ticks or counts per rotation
//const double speed_to_pwm_ratio = 0.00429//0.00235;     //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
//const double min_speed_cmd = 0.0882;           //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

const double left_speed_to_cmd_ratio = 0.000429;     //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double left_min_speed_cmd = -0.983;           //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

const double right_speed_to_cmd_ratio = 0.000765;     //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double right_min_speed_cmd = -1.57;

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s

const double max_speed = 0.4;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor

// PID Parameters
//const double PID_left_param[] = { 0.05, 0, 0.1 }; //Respectively Kp, Ki and Kd for left motor PID
//const double PID_right_param[] = { 0.05, 0, 0.1 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

//PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
//PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

JrkG2I2C leftJrk(11);
JrkG2I2C rightJrk(12);

// Pins
/*const uint8_t R_PWM = 44;
  const uint8_t R_FORW = 23;
  const uint8_t R_BACK = 24;
  const uint8_t L_FORW = 26;
  const uint8_t L_BACK = 25;
  const uint8_t L_PWM = 46;*/

ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication

  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

void setup() {
  Wire.begin();

  /*
    pinMode(L_PWM, OUTPUT);
    pinMode(L_FORW, OUTPUT);
    pinMode(L_BACK, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(R_FORW, OUTPUT);
    pinMode(R_BACK, OUTPUT);*/

  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic

  //setting motor speeds to zero
  stop();

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

void loop() {
  nh.spinOnce();
  if ((millis() - lastMilli) >= LOOPTIME)
  { // enter timed loop
    lastMilli = millis();

    if (abs(pos_left) < 2) {                                                  //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left = ((pos_left / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of left wheel
    }

    if (abs(pos_right) < 2) {                                                 //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
      speed_act_right = ((pos_right / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of right wheel
    }

    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    //    PID_leftMotor.Compute();
    // compute PWM value for left motor. Check constant definition comments for more information.

    PWM_leftMotor = constrain(((speed_req_left + sgn(speed_req_left) * left_min_speed_cmd ) / left_speed_to_cmd_ratio) + (speed_cmd_left / left_speed_to_cmd_ratio), 0, 4095);

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      leftJrk.stopMotor();
    }
    else if (speed_req_left == 0) {                       //Stopping
      leftJrk.stopMotor();
    }
    else {                                               //Driving
      leftJrk.setTarget(PWM_leftMotor);
    }

    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);
    //    PID_rightMotor.Compute();
    // compute PWM value for right motor. Check constant definition comments for more information.
    PWM_rightMotor = constrain(((speed_req_right + sgn(speed_req_right) * right_min_speed_cmd) / right_speed_to_cmd_ratio) + (speed_cmd_right / right_speed_to_cmd_ratio), 0, 4095);

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      rightJrk.stopMotor();
    }
    else if (speed_req_right == 0) {                      //Stopping
      rightJrk.stopMotor();
    }
    else {                                                //Driving
      rightJrk.setTarget(PWM_leftMotor);
    }

    if ((millis() - lastMilli) >= LOOPTIME) {     //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535) {
      noCommLoops = noCommLoopMax;
    }
    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
}
