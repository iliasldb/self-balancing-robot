#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

const float ALPHA = 0.98;

float angle     = 0.0;
unsigned long last_time = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1) delay(10);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Calibration — average gyro bias over 200 samples at startup
    Serial.println("Calibrating gyro — hold the board still...");
    float gyro_bias = 0.0;
    for (int i = 0; i < 200; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gyro_bias += g.gyro.y;   // AX tilt = GY rotation
        delay(5);
    }
    gyro_bias /= 200.0;
    Serial.print("Gyro bias: ");
    Serial.println(gyro_bias, 5);

    // Store bias in a global so loop() can use it
    // (we'll handle this with a simple global below)
    last_time = micros();
}

// Gyro bias measured at startup
float gyro_bias = 0.0;

void loop() {
    unsigned long now = micros();
    float dt = (now - last_time) / 1e6;
    last_time = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Correct axis: AX/AZ for forward tilt, GY for rotation rate
    float accel_angle = atan2(a.acceleration.x, -a.acceleration.z) * 180.0 / PI;
    float gyro_rate   = g.gyro.y - gyro_bias;
    float gyro_angle  = angle + gyro_rate * dt * 180.0 / PI;

    // Complementary filter
    angle = ALPHA * gyro_angle + (1 - ALPHA) * accel_angle;

    Serial.print("Angle: ");
    Serial.print(angle, 2);
    Serial.print(" deg  |  Accel: ");
    Serial.print(accel_angle, 2);
    Serial.print(" deg  |  Gyro rate: ");
    Serial.println(gyro_rate, 4);

    delay(10);
}