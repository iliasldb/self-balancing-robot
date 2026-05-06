#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

// Complementary filter
float angle     = 0.0;
float ALPHA     = 0.98;
unsigned long last_time = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1);
    }

    Serial.println("MPU6050 ready.");
    last_time = micros();
}

void loop() {
    // Get time delta in seconds
    unsigned long now = micros();
    float dt = (now - last_time) / 1e6;
    last_time = now;

    // Read sensor
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Accelerometer angle
    float accel_angle = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;

    // Gyroscope integration
    float gyro_angle = angle + g.gyro.x * dt * 180.0 / PI;

    // Complementary filter
    angle = ALPHA * gyro_angle + (1 - ALPHA) * accel_angle;

    Serial.print("Angle: ");
    Serial.print(angle, 2);
    Serial.print(" deg  |  Accel: ");
    Serial.print(accel_angle, 2);
    Serial.print("  |  Gyro rate: ");
    Serial.println(g.gyro.x, 4);

    delay(10);   // 100 Hz
}