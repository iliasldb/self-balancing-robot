'''
Self-Balancing Robot Simulation

Simulates an inverted pendulum system and compares control strategies under realistic sensor noise"
1. No Control
2. PID Control - raw noisy sensor
3. PID Control - complementary filter
4. PID Control - Kalman filter
5. LQR Control - Kalman filter
'''

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

#  Physical parameters
g = 9.81
l = 0.5
m = 0.1
M = 1.0

# Sensor noise parameters 
ACCEL_NOISE_STD = 0.05    # accelerometer noise std (radians)
GYRO_NOISE_STD  = 0.01    # gyroscope noise std (rad/s)
GYRO_BIAS       = 0.02    # constant gyro drift (rad/s)

#  Equations of motion 

def pendulum(t, y, F):
    theta, theta_dot, x, x_dot = y
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)
    denom = M + m - m * cos_t**2

    theta_ddot = ((M + m) * g * sin_t - cos_t * F) / (l * denom)
    x_ddot     = (F + m * l * theta_dot**2 * sin_t
                  - m * g * sin_t * cos_t) / denom

    return [theta_dot, theta_ddot, x_dot, x_ddot]


def get_noisy_measurement(true_theta, true_theta_dot):
    """Returns noisy accelerometer angle (rad) and noisy gyro rate (rad/s)."""
    accel_angle = true_theta + np.random.normal(0, ACCEL_NOISE_STD)
    gyro_rate   = true_theta_dot + np.random.normal(0, GYRO_NOISE_STD) + GYRO_BIAS
    return accel_angle, gyro_rate


# Kalman filter helper
def make_kalman(dt):
    """Returns Kalman filter matrices."""
    A_kf = np.array([[1, dt],
                     [dt * (M + m) * g / (M * l), 1]])
    Q_kf = np.array([[0.001, 0],
                     [0,     0.003]])
    R_kf = np.array([[0.05]])
    H    = np.array([[1, 0]])
    return A_kf, Q_kf, R_kf, H


def kalman_step(x_hat, P, measurement, A_kf, Q_kf, R_kf, H):
    """One Kalman predict + update step."""
    x_hat = A_kf @ x_hat
    P     = A_kf @ P @ A_kf.T + Q_kf
    S      = H @ P @ H.T + R_kf
    K_gain = P @ H.T @ np.linalg.inv(S)
    x_hat  = x_hat + K_gain @ (measurement - H @ x_hat)
    P      = (np.eye(2) - K_gain @ H) @ P
    return x_hat, P


#  Open loop

def simulate_open_loop(y0, t_end=2.0):
    t_eval = np.linspace(0, t_end, 300)
    sol    = solve_ivp(
        fun=lambda t, y: pendulum(t, y, 0.0),
        t_span=(0, t_end), y0=y0, t_eval=t_eval, method='RK45',
        events=lambda t, y: abs(y[0]) - np.pi / 2
    )
    return sol.t, sol.y[0] * 180 / np.pi


#PID core

def pid_update(theta_est, integral, prev_error, kp, ki, kd, dt):
    error      = theta_est
    integral   = np.clip(integral + error * dt, -5, 5)
    derivative = (error - prev_error) / dt
    F          = np.clip(kp * error + ki * integral + kd * derivative, -20, 20)
    return F, integral, error


def run_simulation(y0, get_estimate_fn, kp, ki, kd, dt=0.01, t_end=5.0, label=""):
    """
    Generic simulation runner.
    get_estimate_fn(y_cur, angle_state, dt) -> (theta_est_rad, new_angle_state)
    """
    t_vals, y_vals = [0.0], [list(y0)]
    y_cur          = list(y0)
    integral       = 0.0
    prev_error     = y0[0]
    angle_state    = None
    t              = 0.0

    while t < t_end:
        theta_est, angle_state = get_estimate_fn(y_cur, angle_state, dt)

        F, integral, prev_error = pid_update(
            theta_est, integral, prev_error, kp, ki, kd, dt)

        sol   = solve_ivp(lambda t, y: pendulum(t, y, F),
                          (t, t + dt), y_cur, method='RK45')
        y_cur = [sol.y[i][-1] for i in range(4)]
        t    += dt
        t_vals.append(t)
        y_vals.append(list(y_cur))

        if abs(y_cur[0]) > np.pi / 2:
            if label:
                print(f"{label}: fell at t={t:.2f}s")
            break

    return np.array(t_vals), np.array(y_vals)


# Estimator functions
def estimator_noisy(y_cur, state, dt):
    """Raw noisy accelerometer — no filtering."""
    accel_angle, _ = get_noisy_measurement(y_cur[0], y_cur[1])
    return accel_angle, state


def estimator_complementary(y_cur, angle_cf, dt, alpha=0.98):
    """Complementary filter blending gyro and accelerometer."""
    accel_angle, gyro_rate = get_noisy_measurement(y_cur[0], y_cur[1])
    if angle_cf is None:
        angle_cf = accel_angle
    angle_cf = alpha * (angle_cf + gyro_rate * dt) + (1 - alpha) * accel_angle
    return angle_cf, angle_cf


def estimator_kalman(y_cur, kf_state, dt):
    """Kalman filter estimator."""
    A_kf, Q_kf, R_kf, H = make_kalman(dt)
    accel_angle, _       = get_noisy_measurement(y_cur[0], y_cur[1])
    measurement          = np.array([accel_angle])

    if kf_state is None:
        kf_state = (np.array([y_cur[0], y_cur[1]]), np.eye(2) * 0.1)

    x_hat, P   = kalman_step(kf_state[0], kf_state[1],
                              measurement, A_kf, Q_kf, R_kf, H)
    return x_hat[0], (x_hat, P)


# LQR with Kalman

def compute_lqr_gain(Q, R):
    """
    Linearized system around upright equilibrium.
    Sign convention matches pendulum() equations of motion:
    positive F = cart moves right = pendulum pushed left = angle decreases
    """
    A = np.array([
        [0,                       1,  0, 0],
        [(M + m) * g / (M * l),   0,  0, 0],
        [0,                       0,  0, 1],
        [-m * g / M,              0,  0, 0]
    ])

    B = np.array([
        [0          ],
        [1 / (M * l)],   # sign flipped vs before
        [0          ],
        [1 / M      ]
    ])

    P = solve_continuous_are(A, B, Q, R)
    K = (np.linalg.inv(R) @ B.T @ P).flatten()
    return K


def simulate_lqr_kalman(Q, R, y0, t_end=5.0, dt=0.01):
    """
    LQR using Kalman-filtered theta and theta_dot.
    Cart position and velocity use true values
    (no encoder in this simulation).
    """
    K            = compute_lqr_gain(Q, R)
    print(f"K = {K}")
    A_kf, _, _, H = make_kalman(dt)

    # LQR-specific Kalman tuning — smoother than PID version
    Q_kf = np.array([[0.1,  0  ],
                 [0,    0.1]])    # trust model less
    R_kf = np.array([[0.001]])       # trust measurement much more

    x_hat          = np.array([y0[0], y0[1]])
    P_kf           = np.eye(2) * 0.01
    F_prev         = 0.0
    t_vals, y_vals = [0.0], [list(y0)]
    y_cur          = list(y0)
    t              = 0.0

    while t < t_end:
        accel_angle, _ = get_noisy_measurement(y_cur[0], y_cur[1])
        measurement    = np.array([accel_angle])

        x_hat, P_kf = kalman_step(x_hat, P_kf, measurement,
                                   A_kf, Q_kf, R_kf, H)

        state  = np.array([x_hat[0], x_hat[1], y_cur[2], y_cur[3]])
        F_raw  = float(np.clip(K @ state, -50, 50))
        F      = F_raw
        F_prev = F
        
        if t < 0.06:
            print(f"t={t:.3f} | theta_est={x_hat[0]*180/np.pi:.2f}° "
                f"| F_raw={F_raw:.2f} | F={F:.2f} "
                f"| true_theta={y_cur[0]*180/np.pi:.2f}°")

        sol   = solve_ivp(lambda t, y: pendulum(t, y, F),
                          (t, t + dt), y_cur, method='RK45')
        y_cur = [sol.y[i][-1] for i in range(4)]
        t    += dt
        t_vals.append(t)
        y_vals.append(list(y_cur))

        if abs(y_cur[0]) > np.pi / 2:
            print(f"LQR+KF: fell at t={t:.2f}s")
            break
            
    
    return np.array(t_vals), np.array(y_vals)


#Run all simulations 

np.random.seed(42)   # reproducible noise
y0      = [10 * np.pi / 180, 0.0, 0.0, 0.0]
KP, KI, KD = 50, 2, 8

print("Running simulations...")

t_open,  theta_open  = simulate_open_loop(y0)

t_noisy, y_noisy     = run_simulation(y0, estimator_noisy,
                                       KP, KI, KD, label="PID noisy")
t_cf,    y_cf        = run_simulation(y0, estimator_complementary,
                                       KP, KI, KD, label="PID+CF")
t_kf,    y_kf        = run_simulation(y0, estimator_kalman,
                                       KP, KI, KD, label="PID+KF")

t_lqr, y_lqr = simulate_lqr_kalman(
                    np.diag([50, 10, 1, 2]), np.array([[0.1]]), y0)

theta_noisy = y_noisy[:, 0] * 180 / np.pi
theta_cf    = y_cf[:,    0] * 180 / np.pi
theta_kf    = y_kf[:,    0] * 180 / np.pi
theta_lqr   = y_lqr[:,   0] * 180 / np.pi

print("Done. Plotting...")

# Plot 

fig, (ax_fall, ax_pid, ax_lqr) = plt.subplots(3, 1, figsize=(11, 11))
fig.suptitle('Inverted Pendulum — Controller and Filter Comparison',
             fontsize=13, fontweight='bold', y=0.99)

# Plot 1: Open loop fall
ax_fall.plot(t_open, theta_open, color='#A32D2D', linewidth=2)
ax_fall.axhline(0, color='gray', linestyle='--', linewidth=0.8)
ax_fall.set_title('No controller — pendulum falls freely')
ax_fall.set_ylabel('Angle (°)')
ax_fall.set_xlabel('Time (s)')
ax_fall.set_ylim(-95, 95)
ax_fall.set_yticks([-90, -60, -30, 0, 30, 60, 90])
ax_fall.grid(True, alpha=0.3)

#Plot 2: PID comparison on same axes
ax_pid.plot(t_noisy, theta_noisy, color='#E07B00',
            linewidth=1.5, label='PID — no filter (raw noisy sensor)', alpha=0.8)
ax_pid.plot(t_cf,    theta_cf,    color='#185FA5',
            linewidth=1.5, label='PID — complementary filter (α=0.98)')
ax_pid.plot(t_kf,    theta_kf,    color='#3B6D11',
            linewidth=1.5, label='PID — Kalman filter')
ax_pid.axhline(0, color='gray', linestyle='--', linewidth=0.8)
ax_pid.set_title('PID controllers — effect of sensor filtering')
ax_pid.set_ylabel('Angle (°)')
ax_pid.set_xlabel('Time (s)')
ax_pid.set_ylim(-20, 20)
ax_pid.legend(loc='upper right', fontsize=9)
ax_pid.grid(True, alpha=0.3)

# Plot 3: Best PID vs LQR 
ax_lqr.plot(t_kf,  theta_kf,  color='#3B6D11',
            linewidth=1.5, label='PID — Kalman filter')
ax_lqr.plot(t_lqr, theta_lqr, color='#6A0DAD',
            linewidth=1.5, label='LQR — Kalman filter (optimal)')
ax_lqr.axhline(0, color='gray', linestyle='--', linewidth=0.8)
ax_lqr.set_title('PID vs LQR — both using Kalman filter')
ax_lqr.set_ylabel('Angle (°)')
ax_lqr.set_xlabel('Time (s)')
ax_lqr.set_ylim(-20, 20)
ax_lqr.legend(loc='upper right', fontsize=9)
ax_lqr.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('simulation/simulation_result.png', dpi=150)
plt.show()
print("Saved to simulation/simulation_result.png")