// 2 motors, PID, manual control

// pins
int input_1 = 4;
int input_2 = 5;
int input_3 = 6;
int input_4 = 7;
int enable_a = 9;
int enable_b = 10;
int opt_a = 2;
int opt_b = 3;
int ky_023_x = 0;  // analog
int speed_control = 1;  // analog

// const
const int count_opt_change = 15;
const int minOut = 0;
const int maxOut = 255;
const int maxMotorRPM = 60;
const float PID_kP_a = 4.0;
const float PID_kI_a = 0.0;
const float PID_kD_a = 0.0;
const float PID_kP_b = 4.0;
const float PID_kI_b = 0.0;
const float PID_kD_b = 0.0;
const uint32_t PID_dt = 10;
const uint32_t control_dt = 100;

// var
volatile uint32_t RPM_timer_a = 0;
volatile uint32_t RPM_time_delta_a = 0;
volatile uint32_t RPM_timer_b = 0;
volatile uint32_t RPM_time_delta_b = 0;
float PID_integral_a = 0.0;
float PID_prevErr_a = 0.0;
uint32_t PID_timer = 0;
float PID_integral_b = 0.0;
float PID_prevErr_b = 0.0;
uint32_t control_timer = 0;
float speed_coef_a = 1.0;
float speed_coef_b = 1.0;
float global_speed_coef = 0.0;


void setup() {
  pinMode(input_1, OUTPUT);
  pinMode(input_2, OUTPUT);
  pinMode(input_3, OUTPUT);
  pinMode(input_4, OUTPUT);
  pinMode(enable_a, OUTPUT);
  pinMode(enable_b, OUTPUT);

  digitalWrite(input_1, LOW);
  digitalWrite(input_2, HIGH);
  digitalWrite(input_3, HIGH);
  digitalWrite(input_4, LOW);

  attachInterrupt(opt_a - 2, getRPMTimeDeltaA, CHANGE);
  attachInterrupt(opt_b - 2, getRPMTimeDeltaB, CHANGE);

  Serial.begin(9600);
}

void loop() {
  if (millis() - control_timer >= control_dt) {
    int control_y = analogRead(speed_control);
    int y = 0;
    if (control_y < 511) {
      y = map(control_y, 510, 0, 1, 510);
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, HIGH);
      digitalWrite(input_3, HIGH);
      digitalWrite(input_4, LOW);
    }
    if (control_y > 511) {
      y = map(control_y, 512, 1022, 1, 510);
      digitalWrite(input_1, HIGH);
      digitalWrite(input_2, LOW);
      digitalWrite(input_3, LOW);
      digitalWrite(input_4, HIGH);
    }
    if (control_y == 511) {
      digitalWrite(input_1, LOW);
      digitalWrite(input_2, LOW);
      digitalWrite(input_3, LOW);
      digitalWrite(input_4, LOW);
    }
    global_speed_coef = (float)y / 510.0;
    int control_x = analogRead(ky_023_x);
    if (control_x < 511) {
      speed_coef_a = (float)control_x / 510.0;
      speed_coef_b = 1.0;
    }
    if (control_x > 511) {
      speed_coef_a = 1.0;
      speed_coef_b = (float)(1022 - control_x) / 510.0;
    }
    if (control_x == 511) {
      speed_coef_a = 1.0;
      speed_coef_b = 1.0;
    }
    Serial.println(speed_coef_a);
  }
  if (millis() - PID_timer >= PID_dt) {
    float current_RPM_a = computeRPM_a();
    float current_RPM_b = computeRPM_b();
    int speed_a = computePID_a(
      current_RPM_a,
      global_speed_coef * speed_coef_a * maxMotorRPM,
      PID_kP_a,
      PID_kI_a,
      PID_kD_a,
      (float)PID_dt,
      minOut,
      maxOut);
    int speed_b = computePID_b(
      current_RPM_b,
      global_speed_coef * speed_coef_b * maxMotorRPM,
      PID_kP_b,
      PID_kI_b,
      PID_kD_b,
      (float)PID_dt,
      minOut,
      maxOut);
    analogWrite(enable_a, speed_a);
    analogWrite(enable_b, speed_b);
  }
}

void getRPMTimeDeltaA() {
  volatile uint32_t current_time = millis();
  RPM_time_delta_a = current_time - RPM_timer_a;
  RPM_timer_a = current_time;
}

void getRPMTimeDeltaB() {
  volatile uint32_t current_time = millis();
  RPM_time_delta_b = current_time - RPM_timer_b;
  RPM_timer_b = current_time;
}

float computeRPM_a() {
  if (RPM_time_delta_a == 0) {
    return 0.0;
  }
  float RPM = (1.0 / (float)count_opt_change) / ((float)RPM_time_delta_a / 60000.0);
  RPM_time_delta_a = 0.0;
  return RPM;
}

float computeRPM_b() {
  if (RPM_time_delta_b == 0) {
    return 0.0;
  }
  float RPM = (1.0 / (float)count_opt_change) / ((float)RPM_time_delta_b / 60000.0);
  RPM_time_delta_b = 0.0;
  return RPM;
}

int computePID_a(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  PID_integral_a = constrain(PID_integral_a + (float)err * dt * ki, minOut, maxOut);
  float D = (err - PID_prevErr_a) / dt;
  PID_prevErr_a = err;
  return constrain(err * kp + PID_integral_a + D * kd, minOut, maxOut);
}

int computePID_b(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = setpoint - input;
  PID_integral_b = constrain(PID_integral_b + (float)err * dt * ki, minOut, maxOut);
  float D = (err - PID_prevErr_b) / dt;
  PID_prevErr_b = err;
  return constrain(err * kp + PID_integral_b + D * kd, minOut, maxOut);
}
