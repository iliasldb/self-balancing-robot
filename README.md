# Self-Balancing Robot

A two-wheeled self-balancing robot built on an Adafruit Metro M0 Express,
implementing and comparing PID and LQR control strategies — from mathematical
simulation to working hardware.

---

## Project overview

The inverted pendulum is a classic control problem: a rod balanced upright on a
moving cart, stabilized entirely by feedback control. This project implements it
as a two-wheeled robot that balances on its own.

The project follows four stages — simulation first, hardware second:

| Stage | Description | Status |
|-------|-------------|--------|
| 1 | Physics simulation — PID and LQR comparison in Python | ✅ Complete |
| 2 | IMU integration — MPU-6050 angle estimation on Metro M0 | ✅ Complete |
| 3 | Closed-loop PID control on real hardware | 🔧 In progress |
| 4 | LQR on hardware + performance comparison | ⬜ Planned |

---

## Stage 1 — Simulation

Models the full nonlinear dynamics of a cart-pendulum system using Lagrangian
mechanics, then compares control strategies under realistic sensor noise.

![Simulation results](simulation/simulation_result.png)

### Sensor noise model

All controlled simulations include realistic MPU-6050 sensor noise:
- Accelerometer noise: σ = 0.05 rad
- Gyroscope noise: σ = 0.01 rad/s
- Gyroscope bias: 0.02 rad/s (constant drift)

### Controllers and filters compared

**No control** — pendulum falls to 90° in under one second from a 10° initial tilt.

**PID — no filter** — raw noisy accelerometer angle fed directly to PID.
Noisiest response, most oscillation, falls under sustained disturbance.

**PID — complementary filter** — gyroscope and accelerometer blended with
α=0.98. Smoother response than raw sensor, less overshoot than Kalman+PID.

**PID — Kalman filter** — optimal state estimation, but introduces phase lag
that slightly degrades PID derivative term performance.

**LQR — Kalman filter** — optimal control with optimal estimation. Fastest
settling time, least overshoot once gains and filter are properly tuned.

### Key findings

**Complementary filter outperforms Kalman for PID** — the Kalman filter
introduces a small phase lag that degrades the derivative term, increasing
overshoot. The complementary filter's higher gyro weighting (α=0.98) produces
more responsive estimates better suited for PID control.

**LQR requires careful Kalman tuning** — optimal gains computed for a perfect
system become destabilizing with a poorly tuned estimator. The Kalman filter
must trust measurements sufficiently (low R_kf) to track fast angle changes,
otherwise the controller overshoots violently.

**Optimal estimation and optimal control are separate problems** — the best
filter for one controller is not necessarily best for another. PID performs
better with the complementary filter; LQR performs better with a well-tuned
Kalman filter.

### Run the simulation

```bash
pip install numpy matplotlib scipy
python simulation/simulation.py
```

---

## Stage 2 — IMU Integration

Reads angle from an MPU-6050 IMU mounted on the Metro M0 using a complementary
filter that combines gyroscope and accelerometer data.

### The complementary filter

Neither sensor alone is sufficient:
- Accelerometer: accurate long-term but noisy (sensitive to vibration)
- Gyroscope: smooth short-term but drifts over time

The complementary filter blends both — 98% gyroscope for responsiveness,
2% accelerometer for drift correction: angle = 0.98 × (angle + gyro_rate × dt) + 0.02 × accel_angle

Gyro bias is calibrated at startup by averaging 500 samples while stationary.

### Wiring

| MPU-6050 | Metro M0 |
|----------|----------|
| VCC | 3.3V |
| GND | GND |
| SCL | SCL |
| SDA | SDA |

---

## Stage 3 — PID Control on Hardware

Closes the loop: angle from the IMU feeds a PID controller that drives two
DC motors via a TB6612FNG motor driver.

### Control loop (100Hz)
IMU → complementary filter → angle
angle → PID → output (-255 to +255)
output → TB6612FNG → motor PWM
motors → robot moves → angle changes → repeat

### Safety features
- Motors cut automatically if tilt exceeds 30°
- Integral windup guard prevents accumulation during falls
- Fixed 10ms timestep ensures consistent PID derivative term

### Live gain tuning

Gains can be adjusted via serial monitor without re-uploading firmware:
P20.5   → set KP to 20.5
I0.3    → set KI to 0.3
D1.8    → set KD to 1.8
S2.0    → set setpoint to 2.0 degrees

---

## Hardware

| Component | Part | Purpose |
|-----------|------|---------|
| Microcontroller | Adafruit Metro M0 Express | Main controller |
| IMU | MPU-6050 GY-521 | Angle measurement |
| Motor driver | TB6612FNG | Motor control (3.3V logic) |
| Motors | TT DC gear motors 200RPM x2 | Drive wheels |
| Power | 4x AA batteries (6V) | Motor power |

---

## Concepts covered

- Nonlinear dynamics of an inverted pendulum (Lagrangian mechanics)
- PID control — proportional, integral, derivative terms and tuning
- LQR optimal control — state-space, algebraic Riccati equation, Q/R cost matrices
- Complementary filter for IMU sensor fusion
- Gyroscope bias calibration
- Embedded C++ on ARM Cortex-M0 with PlatformIO
- PWM motor control via TB6612FNG H-bridge driver

---

## Dependencies

**Simulation (Python)**
```bash
pip install numpy matplotlib scipy
```

**Firmware (C++)**

Built with PlatformIO. Libraries installed automatically from `platformio.ini`:
- Adafruit MPU6050
- Adafruit Unified Sensor
- Adafruit BusIO

---

## Author
Ilias — EE undergraduate