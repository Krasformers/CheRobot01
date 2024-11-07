/*******************************************************************************************************
* CheRobot01v2 Program
* Created CherepanovVS 01-10-2024
*******************************************************************************************************/
#include <Servo.h>


//---------------------- Initialization: Sensor & Servo Pins
int sensorTrig = 4;
int sensorEcho = 5;
int pinServoHor = 3;
int pinServoVer = 2;
int trimServoHor = 3;
int trimServoVer = -10;
int optSensorL = 7;
int optSensorR = 6;

//---------------------- Initialization: Setting up Servo Angles
Servo myServoHor;
Servo myServoVer;


//---------------------- Initialization: Motors Pins
int motorLeft[2] = { 9, 8 };
int motorRight[2] = { 10, 11 };

//---------------------- Initialization: Constants
int trackWidth = 135;
int wheelRadius = 34;
float countDiskHoles = 20.0;
float pi = 3.1415;
float forwardDistancesAngles[5][2];

//---------------------- Initialization: look around angels
int lookAroundAngels[10][2] = {
  { 90,  90},
  { 60,  90},
  { 30,  90},
  { 30,  150},
  { 60,  150},
  { 90,  150},
  { 120, 150},
  { 150, 150},
  { 150, 90},
  { 120, 90}
};

//---------------------- Initialization: Setup Modules
void setup() {
  //Setup Monitoring
  Serial.begin(9600);

  //Setup Servo
  myServoHor.attach(pinServoHor);
  myServoVer.attach(pinServoVer);

  //Setup Motors
  for (int i = 0; i < 2; i++) {
    pinMode(motorLeft[i], OUTPUT);
    pinMode(motorRight[i], OUTPUT);
  }

  //Setup Sensor
  pinMode(sensorTrig, OUTPUT);
  pinMode(sensorEcho, INPUT);

  //Setup Opt Sensor
  pinMode(optSensorL, INPUT);
  pinMode(optSensorR, INPUT);
}


//---------------------- Initialization: Driver Methods
void brake() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(motorLeft[i], LOW);
    digitalWrite(motorRight[i], LOW);
  }
}

void motorRun(int motorPin, int optSensorPin, int steps) {
  int opt_old = digitalRead(optSensorPin);
  int opt_new = -1;
  int opt = 0;
  while (opt < steps) {
    digitalWrite(motorPin, HIGH);
    delayMicroseconds(1500);
    opt_new = digitalRead(optSensorPin);
    if (opt_new != opt_old) {
      opt += opt_new;
      opt_old = opt_new;
    }
    digitalWrite(motorPin, LOW);
  }
}

void rotate(int motorPin, int optSensorPin, float angle, int trackWidth, int wheelRadius, float countDiskHoles, float pi) {
  float L = 0.0;
  float angleWheel = 0.0;
  int steps = 0;
  L = 2 * pi * trackWidth * (angle / 360);
  angleWheel = (L / (2 * pi * wheelRadius)) * 360;
  steps = angleWheel / (360 / countDiskHoles);
  motorRun(motorPin, optSensorPin, steps);
}

void rotateLeft(float angle) {
  brake();
  rotate(motorLeft[1], optSensorL, angle, trackWidth, wheelRadius, countDiskHoles, pi);
}

void rotateRight(float angle) {
  brake();
  rotate(motorRight[1], optSensorR, angle, trackWidth, wheelRadius, countDiskHoles, pi);
}

void moveForward(float distance) {
  float angleWheel = 0.0;
  int steps = 0;
  int opt_old_r = digitalRead(optSensorR);
  int opt_old_l = digitalRead(optSensorL);
  int opt_new_r = -1;
  int opt_new_l = -1;
  int opt_r = 0;
  int opt_l = 0;
  angleWheel = (distance / (2 * pi * wheelRadius)) * 360;
  steps = angleWheel / (360 / countDiskHoles);
  brake();
  while ((opt_r < steps) & (opt_l < steps)) {
    digitalWrite(motorRight[0], HIGH);
    digitalWrite(motorLeft[0], HIGH);
    delayMicroseconds(1500);
    opt_new_r = digitalRead(optSensorR);
    opt_new_l = digitalRead(optSensorL);
    if ((opt_new_r != opt_old_r) & (opt_new_l != opt_old_l)) {
      opt_r += opt_new_r;
      opt_old_r = opt_new_r;
      opt_l += opt_new_l;
      opt_old_l = opt_new_l;
    }
    digitalWrite(motorRight[0], LOW);
    digitalWrite(motorLeft[0], LOW);
  }
}

void moveBackward(float distance) {
  float angleWheel = 0.0;
  int steps = 0;
  int opt_old_r = digitalRead(optSensorR);
  int opt_old_l = digitalRead(optSensorL);
  int opt_new_r = -1;
  int opt_new_l = -1;
  int opt_r = 0;
  int opt_l = 0;
  angleWheel = (distance / (2 * pi * wheelRadius)) * 360;
  steps = angleWheel / (360 / countDiskHoles);
  brake();
  while ((opt_r < steps) & (opt_l < steps)) {
    digitalWrite(motorRight[1], HIGH);
    digitalWrite(motorLeft[1], HIGH);
    delayMicroseconds(1500);
    opt_new_r = digitalRead(optSensorR);
    opt_new_l = digitalRead(optSensorL);
    if ((opt_new_r != opt_old_r) & (opt_new_l != opt_old_l)) {
      opt_r += opt_new_r;
      opt_old_r = opt_new_r;
      opt_l += opt_new_l;
      opt_old_l = opt_new_l;
    }
    digitalWrite(motorRight[1], LOW);
    digitalWrite(motorLeft[1], LOW);
  }
}

void headMove(int horAngle, int verAngle, int trimServoHor, int trimServoVer) {
  myServoHor.write(horAngle + trimServoHor);
  myServoVer.write(verAngle + trimServoVer);
}

//---------------------- Initialization: Sensor Methods
float calcDistance() {
  long duration;
  float forwardDistance;
  digitalWrite(sensorTrig, LOW);
  delayMicroseconds(20);
  digitalWrite(sensorTrig, HIGH);
  delayMicroseconds(20);
  digitalWrite(sensorTrig, LOW);
  duration = pulseIn(sensorEcho, HIGH);
  forwardDistance = duration / 58;
  return forwardDistance;
}

void lookAround(float forwardDistancesAngles[5][2]) {
  float distancesAngles[10];
  for (int i = 0; i < 10; i++) {
    headMove(lookAroundAngels[i][0], lookAroundAngels[i][1], trimServoHor, trimServoVer);
    delay(200);
    distancesAngles[i] = calcDistance();
  }
  headMove(lookAroundAngels[0][0], lookAroundAngels[0][1], trimServoHor, trimServoVer);
  for (int i = 3; i < 8; i++) {
    forwardDistancesAngles[i - 3][1] = distancesAngles[i];
  }
  for (int i = 2; i > -1; i--) {
    forwardDistancesAngles[2 - i][0] = distancesAngles[i];
  }
  forwardDistancesAngles[3][0] = distancesAngles[9];
  forwardDistancesAngles[4][0] = distancesAngles[8];
}

void robotMove() {
  float distConstForward = 4.0;
  float distConstDownMin = 6.0;
  float distConstDownMax = 10.0;
  lookAround(forwardDistancesAngles);
  if ((forwardDistancesAngles[1][0] > distConstForward) & (forwardDistancesAngles[1][1] > distConstDownMin) & (forwardDistancesAngles[1][1] < distConstDownMax) & (forwardDistancesAngles[2][0] > distConstForward) & (forwardDistancesAngles[2][1] > distConstDownMin) & (forwardDistancesAngles[2][1] < distConstDownMax) & (forwardDistancesAngles[3][0] > distConstForward) & (forwardDistancesAngles[3][1] > distConstDownMin) & (forwardDistancesAngles[3][1] < distConstDownMax)) {
    float minDist = forwardDistancesAngles[1][0];
    if (minDist > forwardDistancesAngles[2][0]) {
      minDist = forwardDistancesAngles[2][0];
    }
    if (minDist > forwardDistancesAngles[3][0]) {
      minDist = forwardDistancesAngles[3][0];
    }
    moveForward((minDist - distConstForward) * 8);
    return;
  }
  if ((forwardDistancesAngles[0][0] > distConstForward) & (forwardDistancesAngles[0][1] > distConstDownMin) & (forwardDistancesAngles[0][1] < distConstDownMax) & (forwardDistancesAngles[1][0] > distConstForward) & (forwardDistancesAngles[1][1] > distConstDownMin) & (forwardDistancesAngles[1][1] < distConstDownMax) & (forwardDistancesAngles[2][0] > distConstForward) & (forwardDistancesAngles[2][1] > distConstDownMin) & (forwardDistancesAngles[2][1] < distConstDownMax)) {
    rotateRight(30.0);
    return;
  }
  if ((forwardDistancesAngles[2][0] > distConstForward) & (forwardDistancesAngles[2][1] > distConstDownMin) & (forwardDistancesAngles[2][1] < distConstDownMax) & (forwardDistancesAngles[3][0] > distConstForward) & (forwardDistancesAngles[3][1] > distConstDownMin) & (forwardDistancesAngles[3][1] < distConstDownMax) & (forwardDistancesAngles[4][0] > distConstForward) & (forwardDistancesAngles[4][1] > distConstDownMin) & (forwardDistancesAngles[4][1] < distConstDownMax)) {
    rotateLeft(30.0);
    return;
  }
  if ((forwardDistancesAngles[0][0] > distConstForward) & (forwardDistancesAngles[0][1] > distConstDownMin) & (forwardDistancesAngles[0][1] < distConstDownMax)) {
    rotateRight(60.0);
    return;
  }
  if ((forwardDistancesAngles[4][0] > distConstForward) & (forwardDistancesAngles[4][1] > distConstDownMin) & (forwardDistancesAngles[4][1] < distConstDownMax)) {
    rotateLeft(60.0);
    return;
  }
  moveBackward(distConstForward * 8.0);
  rotateRight(15.0);
  return;
}


void loop() {
  robotMove();
  delay(200);
}
