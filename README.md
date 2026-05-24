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

> **Current hardware status:** the robot achieves stable upright regulation for
> short intervals (holding within ±1° of vertical with small control effort)
> while tethered. Remaining work is robustness and untethered operation — see
> the [Engineering notes](#engineering-notes--hardware-bring-up) and
> [Changelog](#changelog) for the full bring-up story.

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

### Identifying the IMU axes

Before the filter can work, the IMU's physical mounting orientation has to be
mapped to the tilt axis. On this build, with the sensor mounted as it sits on
the chassis:

- **Z axis points up** (reads ≈ +1g when the robot is vertical)
- **X axis is the forward-tilt direction** (grows as the robot leans forward)
- **Tilt rotation happens about the Y axis** → the **Y gyro (`gy`)** gives the
  angular rate

So the upright angle is computed as `atan2(ax, az)`, and the gyro rate used in
the filter is `gy` (not `gx`). This was determined empirically with an axis
diagnostic sketch rather than assumed — see the engineering notes.

### The complementary filter

Neither sensor alone is sufficient:
- Accelerometer: accurate long-term but noisy (sensitive to motor vibration)
- Gyroscope: smooth short-term but drifts over time

The complementary filter blends both — gyroscope for responsiveness,
accelerometer for drift correction:

```
angle = α × (angle + gyro_rate × dt) + (1 − α) × accel_angle
```

with α between 0.95 and 0.98. Gyro bias is calibrated at startup by averaging
1000 samples while stationary.

### Wiring

| MPU-6050 | Metro M0 |
|----------|----------|
| VCC | 5V (module has onboard 3.3V regulator) |
| GND | GND |
| SCL | SCL |
| SDA | SDA |

The MPU-6050 sits at I2C address `0x68` (or `0x69` if AD0 is pulled high). An
I2C scanner sketch is included for verifying the connection.

---

## Stage 3 — PID Control on Hardware

Closes the loop: angle from the IMU feeds a PID controller that drives two
DC gear motors via an **L298N** dual H-bridge.

### Control loop (100 Hz)

```
IMU → complementary filter → angle estimate
angle → PID (with filtered derivative) → output u (−255 … +255)
u → output remap → motor PWM (compensates stall threshold)
L298N → motors → robot moves → angle changes → repeat
```

### Motor driver — L298N (6-pin control)

The L298N uses six control lines: an enable/PWM pin per motor and a
complementary direction pair per motor.

| Metro M0 pin | L298N pin | Role |
|--------------|-----------|------|
| 5 (PWM) | ENA | Motor A speed |
| 3 | IN1 | Motor A direction |
| 4 | IN2 | Motor A direction |
| 8 | IN3 | Motor B direction |
| 9 | IN4 | Motor B direction |
| 10 (PWM) | ENB | Motor B speed |

Speed is the PWM duty on ENA/ENB; direction is set by driving each IN pair
complementarily (one high, one low). The board's ENA/ENB jumpers are removed so
the enables can be PWM-driven from the microcontroller.

### Actuator nonlinearity — output remapping

A practical issue not present in the simulation: the geared DC motors have a
high static-friction threshold. Under load they do not turn at all until the
PWM duty reaches roughly **100/255 (≈40%)**. A raw PID output therefore produces
a large dead zone followed by an abrupt jump to high speed — bang-bang behavior
that no gain tuning can smooth out.

The fix is at the actuator level: the controller output is remapped so that any
non-zero command maps into the motor's usable range `[threshold, 255]`, with a
squared curve so small corrections stay gentle and large corrections still reach
full power. This converts the dead-zone-plus-jump response into a smooth,
proportional one.

### Derivative filtering

The derivative term is computed on the measurement (not the error, to avoid
derivative kick) and passed through a first-order low-pass filter before being
applied. This matters more than it first appears: the filter time constant has
to be short enough to actually act within the robot's natural rocking period
(~1–2 Hz). Over-filtering adds phase lag that makes the derivative term arrive
too late to damp the oscillation — the symptom is a robot that "won't take Kd"
no matter how high the gain is set.

### Safety features

- Motors cut automatically if tilt exceeds the configured limit (currently 50°)
- Integral term is clamped to prevent windup kicks during recovery
- Fixed 10 ms timestep ensures a consistent derivative term

### Live gain tuning

Gains can be adjusted over the serial monitor without re-uploading firmware
(commands are case-insensitive):

```
p20.5   → set Kp to 20.5
i0.3    → set Ki to 0.3
d1.8    → set Kd to 1.8
s2.0    → set setpoint to 2.0 degrees
r       → reset integral / clear safety trip
?       → print current gains and angle
```

---

## Engineering notes / hardware bring-up

The gap between a clean simulation and a balancing robot is mostly debugging.
The issues encountered during bring-up, roughly in order:

1. **No motor response** — traced to a pin-map mismatch: the firmware assumed a
   TB6612-style layout (separate PWM + enable) while the hardware is an L298N
   (enable *is* the PWM). Rewriting the motor stage around the L298N's 6-pin
   model restored control.
2. **IMU reading garbage** — all axes returned −1, i.e. the I2C bus returning
   `0xFF`. A loose connection; confirmed with an I2C scanner, fixed by reseating,
   verified by reading sensible accelerometer values.
3. **Wrong IMU axes** — the initial angle formula assumed a Y/Z tilt plane, but
   the sensor is mounted with X forward and Z up. Switched to `atan2(ax, az)`
   and the `gy` gyro channel.
4. **Diverging oscillation** — the derivative term had the wrong sign relative
   to the motion, so it pumped energy into the oscillation instead of removing
   it. The robot would hold briefly near vertical (where corrections are tiny)
   then swing apart — a clear signature of inverted velocity feedback.
5. **"Won't take Kd"** — heavy derivative filtering (α = 0.95) added ~200 ms of
   lag, longer than a quarter of the rocking period, so the damping force
   arrived too late. Reducing the filter strength let the derivative term
   actually damp.
6. **Bang-bang motor behavior** — the 40% stall threshold meant corrections did
   nothing until they were large, then slammed. Solved with the output remap
   described above.
7. **Integral windup kicks** — Ki accumulated during stable periods and released
   sudden kicks. Clamping the integral term to a small range fixed it.

Net result: stable upright regulation for short intervals while tethered. The
remaining limitation is the USB tether (cable drag injects disturbances) and
inconsistent manual release — addressed next by moving to battery power and,
later, a Bluetooth link for untethered tuning.

---

## Hardware

| Component | Part | Purpose |
|-----------|------|---------|
| Microcontroller | Adafruit Metro M0 Express | Main controller |
| IMU | MPU-6050 (GY-521) | Angle measurement |
| Motor driver | L298N dual H-bridge | Motor control |
| Motors | TT DC gear motors, 200 RPM ×2 | Drive wheels |
| Power | Battery pack (motor supply) | Motor power |

---

## Concepts covered

- Nonlinear dynamics of an inverted pendulum (Lagrangian mechanics)
- PID control — proportional, integral, derivative terms and tuning
- Derivative-on-measurement with low-pass filtering, and the effect of filter
  phase lag on damping
- LQR optimal control — state-space, algebraic Riccati equation, Q/R cost matrices
- Complementary filter for IMU sensor fusion
- IMU axis identification and gyroscope bias calibration
- Actuator nonlinearity: stall-threshold compensation via output remapping
- Integral anti-windup
- Embedded C++ on ARM Cortex-M0 with PlatformIO
- PWM motor control via an L298N H-bridge

---

## Repository contents

| Path | Description |
|------|-------------|
| `simulation/` | Python cart-pendulum simulation (PID/LQR comparison) |
| `firmware/` | PlatformIO project — main balancing firmware |
| `firmware/` (diagnostics) | Helper sketches: I2C scanner, IMU axis check, motor threshold finder |

---

## Dependencies

**Simulation (Python)**
```bash
pip install numpy matplotlib scipy
```

**Firmware (C++)**

Built with PlatformIO for the Adafruit Metro M0. The core firmware talks to the
MPU-6050 directly over I2C (`Wire`), so no sensor library is strictly required;
optional Adafruit MPU6050 / Unified Sensor / BusIO libraries can be added via
`platformio.ini` if preferred.

---

## Author
Ilias Lahdab — Electrical Engineering undergraduate