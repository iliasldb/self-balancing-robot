# Self-Balancing Robot

A two-wheeled self-balancing robot built on an Adafruit Metro M0 Express,
implementing and comparing PID and LQR control strategies — from mathematical
simulation to working hardware.

Developed during the summer before starting a Master's in Robotics at UC San Diego,
as a hands-on introduction to control theory, embedded C++, and sensor fusion.

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
mechanics, then compares three scenarios.

![Simulation results](simulation/simulation_result.png)

### Controllers compared

**No control** — starting from 10°, the pendulum falls to 90° in under one second.
Demonstrates why active control is necessary.

**PID controller** — manually tuned gains (Kp=50, Ki=2, Kd=8) stabilize the angle
with moderate oscillation. No knowledge of system physics required — gains are found
by feel and iteration.

**LQR controller** — optimal gains computed automatically via the continuous-time
algebraic Riccati equation. Given a cost matrix expressing priorities (Q penalizes
state error, R penalizes control effort), the math finds the best possible gains.
Result: faster settling and less overshoot than PID.

### Key insight

PID requires manual tuning. LQR requires an accurate system model but computes
optimal gains automatically. In simulation LQR wins clearly — the hardware stages
will test how well this holds with real-world noise and modeling error.

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