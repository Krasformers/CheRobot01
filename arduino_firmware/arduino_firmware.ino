/*******************************************************************************************************
* CheRobot01 Program
* Created CherepanovVS 01-10-2024
*******************************************************************************************************/
#include <Servo.h>


//---------------------- Initialization: Sensor & Servo Pins
int sensorTrig = 2;
int sensorEcho = 3;
int pinServo = 4;
int trimServoAngle = -10;


//---------------------- Initialization: Setting up Servo Angles
int servoAngles[5] = {30, 60, 90, 120, 150};
Servo myServo;


//---------------------- Initialization: Motors Pins
int motor_left[2] = {9, 8};
int motor_right[2] = {10, 11};


//---------------------- Initialization: Setup Modules
void setup() {
  //Setup Monitoring
  Serial.begin(9600);

  //Setup Servo
  myServo.attach(pinServo);

  //Setup Motors
  
  for (int i = 0; i < 2; i++)
  {
    pinMode(motor_left[i], OUTPUT);
    pinMode(motor_right[i], OUTPUT);
  }

  //Setup Sensor
  pinMode(sensorTrig, OUTPUT);
  pinMode(sensorEcho, INPUT);
}


//---------------------- Initialization: Driver Methods
void brake(int timeout)
{
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], LOW);
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], LOW);
  delay(timeout);
}

void forward(int timeout)
{
  brake(0);
  digitalWrite(motor_left[0], HIGH);
  digitalWrite(motor_right[0], HIGH);
  delay(timeout);
}

void backward(int timeout)
{
  brake(0);
  digitalWrite(motor_left[1], HIGH);
  digitalWrite(motor_right[1], HIGH);
  delay(timeout);
}

void left(int timeout)
{
  brake(0);
  digitalWrite(motor_left[0], HIGH);
  digitalWrite(motor_right[1], HIGH);
  delay(timeout);
}

void right(int timeout)
{
  brake(0);
  digitalWrite(motor_left[1], HIGH);
  digitalWrite(motor_right[0], HIGH);
  delay(timeout);
}

void shiftLeft(int timeout)
{
  brake(0);
  digitalWrite(motor_left[1], HIGH);
  delay(timeout);
}

void shiftRight(int timeout)
{
  brake(0);
  digitalWrite(motor_right[1], HIGH);
  delay(timeout);
}


//---------------------- Initialization: Sensor Methods
float calcDistance(int angle)
{
  long duration;
  float forwardDistance;
  myServo.write(angle);
  delay(500);
  digitalWrite(sensorTrig, LOW);
  delayMicroseconds(20);
  digitalWrite(sensorTrig, HIGH);
  delayMicroseconds(20);
  digitalWrite(sensorTrig, LOW);
  duration = pulseIn(sensorEcho, HIGH);
  forwardDistance = duration / 58;
  return forwardDistance;
 }

int lookAround(int servoAngles[], int trimServoAngle)
{
  float maxDist;
  int maxDistIdx;
  int servoAnglesCount = 5;
  float lookDist[servoAnglesCount];
  for (int i = 0; i < servoAnglesCount; i++)
  {
    lookDist[i] = calcDistance(servoAngles[i]);
  }
  maxDist = lookDist[0];
  maxDistIdx = 0;
  for (int i = 1; i < servoAnglesCount; i++)
  {
    if (maxDist < lookDist[i])
    {
      maxDist = lookDist[i];
      maxDistIdx = i;
    }
  }
  if (maxDist < 20.0)
  {
    return -1;
  }
  return maxDistIdx;
}

void loop() {
  int fullMotorWorkTime = 500;
  int moveDirection;
  brake(0);
  moveDirection = lookAround(servoAngles, trimServoAngle);
  Serial.println(moveDirection);
  switch (moveDirection) {
    case -1:
      backward(fullMotorWorkTime);
      break;
    case 0:
      shiftRight(fullMotorWorkTime);
      shiftRight(fullMotorWorkTime);
      forward(fullMotorWorkTime);
      break;
    case 1:
      shiftRight(fullMotorWorkTime);
      forward(fullMotorWorkTime);
      break;
    case 2:
      forward(fullMotorWorkTime);
      forward(fullMotorWorkTime);
      break;
    case 3:
      shiftLeft(fullMotorWorkTime);
      forward(fullMotorWorkTime);
      break;
    case 4:
      shiftLeft(fullMotorWorkTime);
      shiftLeft(fullMotorWorkTime);
      forward(fullMotorWorkTime);
      break;
    default:
      brake(0);
      break;
    brake(0);
  }
}
