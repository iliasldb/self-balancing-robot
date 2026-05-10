/*
 * Self-Balancing Robot — Stage 3: PID Control on Hardware
 * ========================================================
 * Implements a closed-loop PID controller on the Adafruit Metro M0 Express.
 * 
 * Hardware:
 *   - Adafruit Metro M0 Express (SAMD21, 48MHz)
 *   - MPU-6050 IMU (I2C) — angle estimation via complementary filter
 *   - TB6612FNG dual motor driver
 *   - 2x TT DC gear motors (6V, 200RPM)
 *   - 4x AA batteries (6V)
 * 
 * Control loop (100Hz):
 *   1. Read IMU → compute angle via complementary filter
 *   2. Compute PID output from angle error
 *   3. Drive motors proportional to PID output
 *   4. Cut motors if angle exceeds safety threshold
 * 
 * Live tuning via serial monitor:
 *   Send P20.5 to set KP, I0.3 for KI, D1.8 for KD, S2.0 for setpoint
 * 
 * Author : Ilias
 * Date   : May 2026
 */

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

//Pin definitions
#define PWMA 5 //Motor A speed control
#define AIN1 3 //Motor A direction control
#define AIN2 4 //Motor A direction control
#define PWMB 10 //Motor B speed control
#define BIN1 8 //Motor B direction control
#define BIN2 9 //Motor B direction control
#define STBY 6 //TB6612FNG standby pin

//PID control parameters
float Kp = 20.0; //Proportional gain
float Ki = 0.5; //Integral gain
float Kd = 1.5; //Derivative gain

//Setpoint for balancing (target angle)
float setpoint = 0.0; //Target angle (upright position)

//Complementary filter parameters
const float alpha = 0.98; //Filter coefficient
float angle = 0.0; //Estimated angle
float gyro_biais = 0.0; //Gyro bias

//PID state variables
float integral = 0.0; //Integral term
float previous_error = 0.0; //Previous error for derivative term

//Timing
unsigned long previous_time = 0; //Previous time for PID calculation
const float DT = 0.01; //Time step (10 ms)

//Safety limits
const float FALL_ANGLE = 30.0; //Angle at which the robot is considered to have fallen

Adafruit_MPU6050 mpu;

/*
    MOTOR CONTROL FUNCTIONS
*/

void motors_init() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); //Take the motor driver out of standby mode
}

void motors_stop(){
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}

void motors_drive(float output){
    //Output is a value between -255 and 255
    //positive output drives forward, negative output drives backward
    int speed = (int)constrain(abs(output), 0, 255);
    bool forward = output > 0;

    //Motor A control
    digitalWrite(AIN1, forward ? HIGH : LOW);
    digitalWrite(AIN2, forward ? LOW : HIGH);
    analogWrite(PWMA, speed);

    //Motor B control   
    digitalWrite(BIN1, forward ? HIGH : LOW);
    digitalWrite(BIN2, forward ? LOW : HIGH);
    analogWrite(PWMB, speed);
}

/*
    IMU INITIALIZATION AND DATA PROCESSING FUNCTIONS
*/

void calibrate_gyro(){
    Serial.println("Calibrating gyro... Keep the robot still.");
    float biais = 0.0;
    for (int i = 0; i < 500; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        biais += g.gyro.y; //Assuming the robot is perfectly still, the gyro reading should be zero. Any average offset is considered bias.
        delay(4);
    }
    gyro_biais = biais / 500.0;
    Serial.print("Gyro bias: ");
    Serial.println(gyro_biais, 5);
}

float read_angle(float dt){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Calculate angle from accelerometer (in degrees)
    float acc_angle = atan2(a.acceleration.x, -a.acceleration.z) * 180.0 / PI;

    //Calculate angle from gyro (in degrees)
    float gyro_rate = g.gyro.y - gyro_biais; //Remove bias
    float gyro_angle = angle + gyro_rate * dt *180.0 / PI; //Integrate gyro rate to get angle

    //Complementary filter to combine accelerometer and gyro data
    angle = alpha * gyro_angle + (1 - alpha) * acc_angle;

    return angle;
}

/*
    PID CONTROL FUNCTION
*/

float compute_pid(float current_angle, float dt){
    float error = current_angle - setpoint; //Calculate error

    //Integral term calculation with anti-windup
    integral += error * dt; //Update integral term
    integral = constraint(integral, -50, 50);

    //derivative term calculation
    float derivative = (error - previous_error) / dt; //Calculate derivative term
    previous_error = error; //Update previous error

    return Kp * error + Ki * integral + Kd * derivative; //Calculate PID output
}

/*
    SETUP AND MAIN LOOP
*/

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10); //Wait for serial connection

    //Initialize IMU
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    //Initialize motors
    motors_init();
    motors_stop();

    //Calibrate gyro
    calibrate_gyro();

    Serial.println("Setup complete. Starting main loop.");
    delay(2000);

    last_time = micros();
}

void loop(){
    //Enforce a consistent loop timing
    unsigned long now = micros();
    if ((now - previous_time) < (DT * 1e6)) return; //Wait until the next time step
    float dt = (now - previous_time) / 1e6; //Calculate actual time step in seconds
    previous_time = now; //Update previous time

    //read current angle from IMU
    angle = read_angle(dt);

    //safety check: if the robot has fallen, stop the motors and reset PID state
    if (abs(angle) > FALL_ANGLE) {
        Serial.println("Robot has fallen! Stopping motors.");
        motors_stop();
        integral = 0.0; //Reset integral term
        previous_error = 0.0; //Reset previous error
        return; //Skip the rest of the loop
    }

    //Compute PID output
    float output = compute_pid(angle, dt);

    check_serial_commands();
    //Drive motors based on PID output
    motors_drive(output);

    Serial.print("Angle: "); Serial.print(angle, 2);
    Serial.print(" | PID: "); Serial.println(output, 2);
    Serial.print(" | Err: "); Serial.print(angle - setpoint, 2);
}

void check_serial_commands(){
    if (Serial.available() > 0) {
       char cmd = Serial.read();
        float val = Serial.parseFloat();
        switch (cmd) {
            case 'P': KP = val; Serial.print("KP="); Serial.println(KP); break;
            case 'I': KI = val; Serial.print("KI="); Serial.println(KI); break;
            case 'D': KD = val; Serial.print("KD="); Serial.println(KD); break;
            case 'S': SETPOINT = val; Serial.print("SP="); Serial.println(SETPOINT); break;
    }
}