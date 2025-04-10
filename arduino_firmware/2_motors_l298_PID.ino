// 2 motors, PID, manual control

// pins
int input1 = 4;
int input2 = 5;
int input3 = 6;
int input4 = 7;
int enableA = 9;
int enableB = 10;
int optA = 2;
int optB = 3;
int ky023X = 0;  // analog
int speedControl = 1;  // analog

// const
const int countOptChanges = 15;
const int minOut = 0;
const int maxOut = 255;
const int maxMotorRPM = 60;
const float pidPA = 4.0;
const float pidIA = 0.0;
const float pidDA = 0.0;
const float pidPB = 4.0;
const float pidIB = 0.0;
const float pidDB = 0.0;
const uint32_t pidDt = 10;
const uint32_t controlDt = 100;

// var
volatile uint32_t interruptTimeA = 0;
volatile uint32_t interruptTimeB = 0;
volatile uint32_t interruptDtA = 0;
volatile uint32_t interruptDtB = 0;
float integralA = 0.0;
float integralB = 0.0;
float prevErrA = 0.0;
float prevErrB = 0.0;
float speedCoefA = 1.0;
float speedCoefB = 1.0;
float speedCoefGlobal = 0.0;
uint32_t pidTime = 0;
uint32_t controlTime = 0;


void setup() {
  pinMode(input1, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(input3, OUTPUT);
  pinMode(input4, OUTPUT);
  pinMode(enableA, OUTPUT);
  pinMode(enableB, OUTPUT);

  digitalWrite(input1, LOW);
  digitalWrite(input2, HIGH);
  digitalWrite(input3, HIGH);
  digitalWrite(input4, LOW);

  attachInterrupt(digitalPinToInterrupt(optA), getInterruptDtA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(optB), getInterruptDtB, CHANGE);
}

void loop() {
  if (millis() - controlTime >= controlDt) {
    int controlY = analogRead(speedControl);
    int y = 0;
    if (controlY < 511) {
      y = map(controlY, 510, 0, 1, 510);
      digitalWrite(input1, LOW);
      digitalWrite(input2, HIGH);
      digitalWrite(input3, HIGH);
      digitalWrite(input4, LOW);
    }
    if (controlY > 511) {
      y = map(controlY, 512, 1022, 1, 510);
      digitalWrite(input1, HIGH);
      digitalWrite(input2, LOW);
      digitalWrite(input3, LOW);
      digitalWrite(input4, HIGH);
    }
    if (controlY == 511) {
      digitalWrite(input1, LOW);
      digitalWrite(input2, LOW);
      digitalWrite(input3, LOW);
      digitalWrite(input4, LOW);
    }
    speedCoefGlobal = (float)y / 510.0;
    int controlX = analogRead(ky023X);
    if (controlX < 511) {
      speedCoefA = (float)controlX / 510.0;
      speedCoefB = 1.0;
    }
    if (controlX > 511) {
      speedCoefA = 1.0;
      speedCoefB = (float)(1022 - controlX) / 510.0;
    }
    if (controlX == 511) {
      speedCoefA = 1.0;
      speedCoefB = 1.0;
    }
  }
  if (millis() - pidTime >= pidDt) {
    float currentRpmA = computeRpmA((float)countOptChanges);
    float currentRpmB = computeRpmB((float)countOptChanges);
    int speedA = computePID(currentRpmA, speedCoefGlobal * speedCoefA * maxMotorRPM, pidPA, pidIA, pidDA, (float)pidDt, integralA, prevErrA, minOut, maxOut);
    int speedB = computePID(currentRpmB, speedCoefGlobal * speedCoefB * maxMotorRPM, pidPB, pidIB, pidDB, (float)pidDt, integralB, prevErrB, minOut, maxOut);
    analogWrite(enableA, speedA);
    analogWrite(enableB, speedB);
  }
}

void getInterruptDtA() {
  volatile uint32_t currentTime = millis();
  interruptDtA = currentTime - interruptTimeA;
  interruptTimeA = currentTime;
}

void getInterruptDtB() {
  volatile uint32_t currentTime = millis();
  interruptDtB = currentTime - interruptTimeB;
  interruptTimeB = currentTime;
}

float computeRpmA(float countOptChanges) {
  if (interruptDtA == 0) {
    return 0.0;
  }
  
  float RPM = (1.0 / countOptChanges) / ((float)interruptDtA / 60000.0);
  detachInterrupt(digitalPinToInterrupt(optA));
  interruptDtA = 0;
  attachInterrupt(digitalPinToInterrupt(optA), getInterruptDtA, CHANGE);
  return RPM;
}

float computeRpmB(float countOptChanges) {
  if (interruptDtB == 0) {
    return 0.0;
  }
  
  float RPM = (1.0 / countOptChanges) / ((float)interruptDtB / 60000.0);
  detachInterrupt(digitalPinToInterrupt(optB));
  interruptDtB = 0;
  attachInterrupt(digitalPinToInterrupt(optB), getInterruptDtB, CHANGE);
  return RPM;
}

int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, float& integral, float& prevErr, int minOut, int maxOut) {
  float err = setpoint - input;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}
