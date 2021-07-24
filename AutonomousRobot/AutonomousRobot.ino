#include "ICM_20948.h"
#define SERIAL_PORT Serial
#define SPI_PORT SPI
#define CS_PIN 2

#define WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

#include <JrkG2.h>
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include <math.h>

double q0, q1, q2, q3 = 0;
double gx, gy, gz, ax, ay, az = 0;

//initializing all the variables
#define LOOPTIME                      100 //115     //Looptime in millisecond (10hz)
int counter = 0;
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

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu/data", &imu_msg);
char odom[] = "/imu0";

void setup() {
  Wire.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      delay(500);
    } else {
      initialized = true;
    }
  }

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
  nh.advertise(imu_pub);

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

  bool success = true; // Use success to show if the DMP configuration was successful
  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

}

void updateIMU() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

      // Scale to +/- 1
      q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));



      /*SERIAL_PORT.print(F("{\"quat_w\":"));
        SERIAL_PORT.print(q0, 3);
        SERIAL_PORT.print(F(", \"quat_x\":"));
        SERIAL_PORT.print(q1, 3);
        SERIAL_PORT.print(F(", \"quat_y\":"));
        SERIAL_PORT.print(q2, 3);
        SERIAL_PORT.print(F(", \"quat_z\":"));
        SERIAL_PORT.print(q3, 3);
        SERIAL_PORT.println(F("}"));*/
    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
    {
      gx = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
      gy = (float)data.Raw_Gyro.Data.Y;
      gz = (float)data.Raw_Gyro.Data.Z;
    }

    if ((data.header & DMP_header_bitmap_Accel) > 0) // Check the packet contains Accel data
    {
      ax = (float)data.Raw_Accel.Data.X; // Extract the raw accelerometer data
      ay = (float)data.Raw_Accel.Data.Y;
      az = (float)data.Raw_Accel.Data.Z;
    }
  }
}

void loop() {
  nh.spinOnce();

  //IMU update
  updateIMU();


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
    publishIMU(LOOPTIME);

  }






}
