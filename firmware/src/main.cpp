/*
 * Self-Balancing Robot - PID Controller (rev 4, L298N wiring corrected)
 * ---------------------------------------------------------------------
 * Target board : Adafruit Metro M0 (SAMD21)
 * IMU          : MPU-6050 over I2C
 * Motor driver : L298N (standard 6-pin control: ENA, IN1, IN2, IN3, IN4, ENB)
 *
 * L298N wiring (6 control wires, NOT 8):
 *   Metro pin 5  (PWM) -> ENA   (Motor A speed)
 *   Metro pin 3        -> IN1   (Motor A direction)
 *   Metro pin 4        -> IN2   (Motor A direction)
 *   Metro pin 8        -> IN3   (Motor B direction)
 *   Metro pin 9        -> IN4   (Motor B direction)
 *   Metro pin 10 (PWM) -> ENB   (Motor B speed)
 *
 * On the L298N: speed is the PWM duty on ENA/ENB; direction is set by the
 * complementary IN pairs (IN1/IN2 for A, IN3/IN4 for B).
 *
 * IMU mounting (confirmed from diagnostics):
 *   Z = up, X = forward-tilt axis, rotation about Y -> use gy.
 *   Angle: 0 deg vertical, negative when tilting forward, positive backward.
 */

#include <Arduino.h>
#include <Wire.h>

// ============================================================
//  L298N PIN MAP  (6 pins)
// ============================================================
#define ENA   5    // Motor A speed  (PWM-capable pin)
#define IN1   3    // Motor A dir
#define IN2   4    // Motor A dir
#define IN3   8    // Motor B dir
#define IN4   9    // Motor B dir
#define ENB   10   // Motor B speed  (PWM-capable pin)

// ============================================================
//  USER CONFIGURATION
// ============================================================
float Kp = 8.0f;     // start gentle; raise gradually
float Ki = 0.5f;      // beware overshoot if you enable this!
float Kd = 5.0f;

float filteredD = 0.0f; 

float setpoint = 0.0f;   // vertical reads ~0 deg

const float DT          = 0.01f;     // 100 Hz
const unsigned long DT_US = 10000UL;
const float ALPHA = 0.98f;
const float TIP_OVER_ANGLE = 35.0f;
const float PID_OUT_MAX = 255.0f;
const float PID_OUT_MIN = -255.0f;
//const int MOTOR_DEADBAND = 10;


const uint8_t MPU_ADDR = 0x68;

// ============================================================
//  STATE
// ============================================================
float angle = 0.0f;
float gyroBiasY = 0.0f;
float integralTerm = 0.0f;
float lastMeasurement = 0.0f;
unsigned long lastLoopTime = 0;
bool safetyTripped = false;

// ============================================================
//  MPU-6050 HELPERS
// ============================================================
void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void mpuInit() {
  Wire.begin();
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);
  mpuWrite(0x1A, 0x03);
  mpuWrite(0x1B, 0x00);
  mpuWrite(0x1C, 0x00);
}

void mpuReadAll(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

void calibrateGyro(int samples = 1000) {
  Serial.println(F("Calibrating gyro... keep robot still."));
  long sum = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < samples; i++) {
    mpuReadAll(ax, ay, az, gx, gy, gz);
    sum += gy;
    delay(2);
  }
  gyroBiasY = (float)sum / (float)samples / 131.0f;
  Serial.print(F("Gyro bias Y (deg/s): "));
  Serial.println(gyroBiasY, 4);
}

void initializeAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadAll(ax, ay, az, gx, gy, gz);
  angle = atan2((float)ax, (float)az) * 180.0f / PI;
  Serial.print(F("Initial angle (deg): "));
  Serial.println(angle, 2);
}

float estimateAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadAll(ax, ay, az, gx, gy, gz);
  float gyroRate = (float)gy / 131.0f - gyroBiasY;
  float accelAngle = atan2((float)ax, (float)az) * 180.0f / PI;
  angle = ALPHA * (angle + gyroRate * DT) + (1.0f - ALPHA) * accelAngle;
  return angle;
}

// ============================================================
//  PID CONTROLLER
// ============================================================
float computePID(float measurement) {
  float error = setpoint - measurement;
  float pTerm = Kp * error;

  integralTerm += Ki * error * DT;
  if (integralTerm > PID_OUT_MAX) integralTerm = PID_OUT_MAX;
  if (integralTerm < PID_OUT_MIN) integralTerm = PID_OUT_MIN;

  float dMeasurement = (measurement - lastMeasurement) / DT;
  const float D_FILTER = 0.6f;
  filteredD = D_FILTER * filteredD + (1.0f - D_FILTER) * dMeasurement;

  float dTerm = Kd * filteredD;
  lastMeasurement = measurement;

  const float I_MAX = 40.0f;

  if (integralTerm > I_MAX) integralTerm = I_MAX;
  if (integralTerm < -I_MAX) integralTerm = -I_MAX;

  float output = pTerm + integralTerm + dTerm;
  if (output > PID_OUT_MAX) output = PID_OUT_MAX;
  if (output < PID_OUT_MIN) output = PID_OUT_MIN;
  return output;
}

// ============================================================
//  MOTOR CONTROL  (L298N: ENx = PWM speed, INx = direction)
// ============================================================
void setMotorA(int speed, bool forward) {
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  // Direction via complementary IN pair
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
  // Speed via PWM on enable pin
  analogWrite(ENA, speed);
}

void setMotorB(int speed, bool forward) {
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW  : HIGH);
  analogWrite(ENB, speed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Drive both wheels with signed effort u in [-255, 255].
// If wheel B happens to be wired mirror-image to A, change the
// second call to setMotorB(pwmB, !forward).
// Motor B starts slightly earlier than A, so trim A up slightly
const float MOTOR_A_TRIM = 1.08f;   // nudge A to match B
const float MOTOR_B_TRIM = 1.00f;

const float MOTOR_THRESHOLD = 100.0f;  // measured true deadband

void driveMotors(float u) {
  if (fabsf(u) < 1.0f) {
    stopMotors();
    return;
  }

  bool forward = (u > 0.0f);
  float absU = fabsf(u);

  // Boost negative direction slightly to compensate L298N asymmetry
  // Tune NEGATIVE_TRIM between 1.0 and 1.2 until both directions feel equal
  const float NEGATIVE_TRIM = 1.0f;
  if (!forward) absU = constrain(absU * NEGATIVE_TRIM, 0.0f, 255.0f);

  float normalized = absU / 255.0f;
  float curved = normalized * normalized;
  float pwm = 100.0f + curved * 155.0f;
  if (pwm > 255.0f) pwm = 255.0f;

  int pwmA = constrain((int)(pwm * MOTOR_A_TRIM), 0, 255);
  int pwmB = constrain((int)(pwm * MOTOR_B_TRIM), 0, 255);

  setMotorA(pwmA, forward);
  setMotorB(pwmB, forward);
}

// ============================================================
//  LIVE SERIAL TUNING
// ============================================================
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  float val = line.substring(1).toFloat();

  switch (cmd) {
    case 'p': case 'P':
      Kp = val; Serial.print(F("Kp = ")); Serial.println(Kp, 4); break;
    case 'i': case 'I':
      Ki = val; integralTerm = 0.0f; Serial.print(F("Ki = ")); Serial.println(Ki, 4); break;
    case 'd': case 'D':
      Kd = val; Serial.print(F("Kd = ")); Serial.println(Kd, 4); break;
    case 's': case 'S':
      setpoint = val; Serial.print(F("setpoint = ")); Serial.println(setpoint, 4); break;
    case 'r': case 'R':
      integralTerm = 0.0f; safetyTripped = false; Serial.println(F("Reset.")); break;
    case '?':
      Serial.print(F("Kp=")); Serial.print(Kp, 3);
      Serial.print(F(" Ki=")); Serial.print(Ki, 3);
      Serial.print(F(" Kd=")); Serial.print(Kd, 3);
      Serial.print(F(" sp=")); Serial.print(setpoint, 3);
      Serial.print(F(" angle=")); Serial.println(angle, 3);
      break;
    default:
      Serial.println(F("Use p/i/d/s<val>, r, or ?")); break;
  }
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  mpuInit();
  delay(100);
  calibrateGyro();
  initializeAngle();
  lastMeasurement = angle;

  Serial.print(F("Boot angle: ")); Serial.print(angle, 2);
  Serial.print(F("  Setpoint: ")); Serial.print(setpoint, 2);
  Serial.print(F("  Initial error: ")); Serial.println(setpoint - angle, 2);
  Serial.println(F("Ready. Commands: p<val> i<val> d<val> s<val> r ?"));

  lastLoopTime = micros();
}

void loop() {
  handleSerial();

  unsigned long now = micros();
  if (now - lastLoopTime < DT_US) return;
  lastLoopTime += DT_US;

  float theta = estimateAngle();

  if (fabsf(theta - setpoint) > TIP_OVER_ANGLE) {
    if (!safetyTripped) {
      Serial.println(F("SAFETY TRIP: tipped. Send 'r' to reset."));
      safetyTripped = true;
    }
    stopMotors();
    integralTerm = 0.0f;
    return;
  }
  if (safetyTripped) { stopMotors(); return; }

  float u = computePID(theta);
  driveMotors(u);

  static uint8_t telemetryCounter = 0;
  if (++telemetryCounter >= 10) {
    telemetryCounter = 0;
    Serial.print(theta, 2);
    Serial.print('\t');
    Serial.println(u, 1);
  }
}