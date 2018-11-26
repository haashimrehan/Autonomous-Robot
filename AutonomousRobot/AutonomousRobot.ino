#include <MedianFilter.h>  //Average ultrasonic sensor Data

// Motors
// Front Right
const int renA = 6;   //Gray
const int rin1 = 53;  //Light red
const int rin2 = 52;  //Dark Red
// Front Left
const int lenB = 5;   //gray
const int lin3 = 47;  //Black
const int lin4 = 46;  //White
// Rear Right
const int renB = 7;   //Orange
const int rin3 = 51;  //Black
const int rin4 = 50;  //White
// Rear Left
const int lenA = 4;   //Orange
const int lin1 = 49;  //Light Red
const int lin2 = 48;  //Dark Red

// Motor Power
const int maxPower = 190;
int Power = maxPower; 
int leftPower = maxPower;
int rightPower = maxPower;
int FLPower = maxPower;
int FRPower = maxPower;
int RLPower = maxPower;
int RRPower = maxPower;

// Front Ping Sensor
long duration;
long cm = 100;
const int trigPin = 45;  //Gray
const int echoPin = 44;  //Blue
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

#define HALLSEN_A 2 //Front Left
#define HALLSEN_B 19 //Front Right

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
boolean measureRpmA = false;
boolean measureRpmB = false;

float error = 0;
float error1 = 0;
float error2 = 0;
float kp = 50.0;


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

Encoder FL(2);
Encoder FR(19);
Encoder RL(20);
Encoder RR(21);

void setup() {
  // Set all the motor control pins to outputs
  pinMode(lenA, OUTPUT);
  pinMode(lenB, OUTPUT);
  pinMode(lin1, OUTPUT);
  pinMode(lin2, OUTPUT);
  pinMode(lin3, OUTPUT);
  pinMode(lin4, OUTPUT);
  pinMode(renA, OUTPUT);
  pinMode(renB, OUTPUT);
  pinMode(rin1, OUTPUT);
  pinMode(rin2, OUTPUT);
  pinMode(rin3, OUTPUT);
  pinMode(rin4, OUTPUT);

  // Ping Sensor Inputs
  pinMode(trigPin, INPUT);
  pinMode(echoPin, INPUT);

  // Initialize Encoders
  FL.initialize();
  FR.initialize();
  RL.initialize();
  RR.initialize();

  // Attach interrupt at hall sensor A on each rising signal
  attachInterrupt(digitalPinToInterrupt(FL.encoderPin), updateEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(FR.encoderPin), updateEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(RL.encoderPin), updateEncoderRL, RISING);
  attachInterrupt(digitalPinToInterrupt(RR.encoderPin), updateEncoderRR, RISING);
  //attachInterrupt(digitalPinToInterrupt(HALLSEN_B), updateEncoderB, RISING);

  // initialize serial communication:
  Serial.begin(38400);
}

void loop()
{
  pingSense(false);

  FL.updateRPM(false);
  FR.updateRPM(false);
  RL.updateRPM(false);
  RR.updateRPM(false);
  eDrive2(1);

}