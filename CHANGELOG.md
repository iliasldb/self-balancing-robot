# Changelog

All notable changes to this project are documented here.

## [Unreleased] — Hardware bring-up

### Added
- L298N motor driver support with correct 6-pin control model
  (ENA/ENB as PWM speed, IN1–IN4 as complementary direction pairs).
- Motor output remapping to compensate for the geared motors' ~40% static
  stall threshold; converts the dead-zone-plus-jump response into a smooth
  proportional one (squared low-end curve, mapped into `[threshold, 255]`).
- First-order low-pass filter on the derivative term.
- Integral anti-windup clamp to prevent accumulated kicks during recovery.
- Per-direction trim to compensate for L298N H-bridge asymmetry.
- Diagnostic sketches: I2C scanner, IMU axis identifier, motor threshold
  finder, and a minimal motor/serial sanity test.

### Changed
- IMU angle estimation corrected to match the physical sensor mounting:
  upright angle now uses `atan2(ax, az)` and the tilt rate uses the `gy`
  gyro channel (previously assumed a Y/Z tilt plane and `gx`).
- Derivative-term sign corrected — previous sign pumped energy into the
  oscillation rather than damping it.
- Derivative filter strength reduced after finding that heavy filtering
  (~200 ms time constant) added enough phase lag to defeat damping.
- Tilt safety cutoff threshold widened to 50° to allow larger recovery swings
  during tuning.
- Gyro bias calibration uses 1000 stationary samples.

### Fixed
- No motor response: pin map assumed a TB6612-style layout; rewired and
  recoded for the L298N.
- IMU returning all −1 values (I2C bus reading `0xFF`) due to a loose
  connection; verified with the I2C scanner after reseating.

### Status
Stable upright regulation for short intervals while tethered (holds within
about ±1° of vertical with small control effort). Tuning baseline around
Kp = 13, Kd = 4, Ki = 0.5.

### Next
- Move to battery power to remove USB cable drag.
- Add a Bluetooth (HC-05) link for untethered live tuning.
- Re-tune on battery power (load behavior differs from USB-stabilized supply).
- Stage 4: implement LQR on hardware and compare against PID.