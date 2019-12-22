#include <Arduino.h>
#include <MedianFilter.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

int straightSpeed = 150;
int currentAngle = 0;

// Motors
// Right
const int pwmA = 44;   
const int dir1A = 23; 
const int dir2A = 24;  
// Left
const int pwmB = 5;
const int dir1B = 47;
const int dir2B = 46;

// Motor Power
const int maxPower = 190;
int Power = maxPower;
int leftPower = maxPower;
int rightPower = maxPower;

// Front Ping Sensor
long duration;
long cm = 100;
const int trigPin = 45; 
const int echoPin = 44;
int pingStopDistance = 40;
int stillThere = 0;
int tempR = 1;
int tempL = 1;
int tempSpd = 1;
MedianFilter pingFilter(50, cm);
int fcm = cm;
bool stopped = false;

// Encoders
#define ENCODEROUTPUT 330

volatile long encoderValueA = 0; //left
volatile long encoderValueB = 0; //right
long leftEncoder = 0;
long rightEncoder = 0;

int interval = 1000;
long previousMillisA = 0;
long previousMillisB = 0;
long currentMillisA = 0;
long currentMillisB = 0;

int rpmA = 0;   //Calculated RPM of Motors
int rpmB = 0;

float error = 0;

float error1 = 0;
float error2 = 0;
float kp = 50.0;

//Camera
#define YELLOW 1
#define BLUE 2
#define RED 3
#define GREEN 5

int midPos;
float mid = 0, sum = 0;
int high = 117 ; //Mid Range
int low = 109; // Mid Range

class Encoder {
  private:
    int interval = 1000;
    long previousMillis = 0;
    long currentMillis = 0;
    boolean measureRpm = false;

  public:
    int encoderTick;   //Used to drive
    int encoderValue;  //resets for RPM
    int rpm = 0;   //Calculated RPM of Motors
    int encoderPin;

    Encoder(int encoder) {
      encoderPin = encoder;
    }

    void initialize() {
      pinMode(encoderPin, INPUT_PULLUP);
      // Attach interrupt at hall sensor A on each rising signal
      // attachInterrupt(digitalPinToInterrupt(encoderPin), updateEncoder, RISING);
    }

    void updateEncoder()
    {
      encoderValue++;
      encoderTick++;
    }

    void updateRPM(bool Print) {
      // Update RPM value on every second
      currentMillis = millis();
      if (currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;

        rpm = (float)(encoderValue * 60 / 330);

        if (Print) {
          Serial.print(encoderValue);
          Serial.print(" ");
          Serial.print(rpm);
          Serial.println(" RPM");
        }
        encoderValue = 0;
      }
    }
};

void setup() {
  Serial.begin(115200);
  
  // Set all the motor control pins to outputs
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(dir1A, OUTPUT);
  pinMode(dir2A, OUTPUT);
  pinMode(dir1B, OUTPUT);
  pinMode(dir2B, OUTPUT);

  // Ping Sensor Inputs
  pinMode(trigPin, INPUT);
  pinMode(echoPin, INPUT);

  gyroSetup();
  
  // initialize serial communication:
  long start = millis();
  while (millis() - start < 3000) {
    Serial.print(".");
    gyroUpdate();
  }
}

void loop()
{
  pingSense(false);
}
