import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are

g = 9.81  # Acceleration due to gravity (m/s^2)
l = 0.5 # Length of the pendulum (m)
m = 0.1 # Mass of the pendulum (kg)
M = 1.0 # Mass of the cart (kg)

def pendulum(t, y, F):
    theta, theta_dot, x, x_dot = y
    
    sin_t =np.sin(theta)
    cos_t = np.cos(theta)
    denom = M + m * cos_t**2

    theta_ddot= ((M+m)*g*sin_t -cos_t*F)/denom
    x_ddot = (F + m *theta_dot**2*sin_t -m*g*sin_t*cos_t)/denom

    return [theta_dot, theta_ddot, x_dot, x_ddot]

# Initial conditions: slight push from vertical
theta_0     = 10 * np.pi / 180   # 10 degrees in radians
theta_dot_0 = 0.0
x_0         = 0.0
x_dot_0     = 0.0

y0 = [theta_0, theta_dot_0, x_0, x_dot_0]

# Time span
t_start = 0.0
t_end   = 2.0
t_eval  = np.linspace(t_start, t_end, 300)

# Simulate with zero force (no controller)
F = 0.0
sol_open = solve_ivp(
    fun=lambda t, y: pendulum(t, y, F),
    t_span=(t_start, t_end),
    y0=y0,
    t_eval=t_eval,
    method='RK45',
    events=lambda t, y: abs(y[0]) - np.pi/2,  # stop at 90 degrees
    dense_output=True
)

def simulate_pid(kp, ki, kd, y0, t_end=5.0, dt=0.01):
    """Simulate the pendulum with a PID controller."""
    t_vals     = [0.0]
    y_vals     = [list(y0)]
    integral   = 0.0
    prev_error = y0[0]   # initial error = initial angle
    y_current  = list(y0)

    t = 0.0
    while t < t_end:
        theta = y_current[0]

        # PID terms
        error      = theta
        integral  += error * dt
        derivative = (error - prev_error) / dt
        F          = kp * error + ki * integral + kd * derivative

        # Clamp force to realistic motor limits
        F = np.clip(F, -20, 20)

        # One step forward using solve_ivp
        sol = solve_ivp(
            fun=lambda t, y: pendulum(t, y, F),
            t_span=(t, t + dt),
            y0=y_current,
            method='RK45'
        )
        y_current  = [sol.y[i][-1] for i in range(4)]
        prev_error = error
        t         += dt

        t_vals.append(t)
        y_vals.append(list(y_current))

        # Stop if fallen
        if abs(y_current[0]) > np.pi / 2:
            print(f"Fell at t={t:.2f}s")
            break

    return np.array(t_vals), np.array(y_vals)

def linearize_system():
    """
    Linearize the pendulum equations around the upright equilibrium.
    Returns A and B matrices of the linear system: x_dot = Ax + Bu
    """
    # A matrix — how the state evolves naturally (no control)
    A = np.array([
        [0,                    1,  0, 0],
        [(M + m) * g / (M * l), 0, 0, 0],
        [0,                    0,  0, 1],
        [-m * g / M,           0,  0, 0]
    ])

    # B matrix — how control input (force) affects the state
    B = np.array([
        [0],
        [-1 / (M * l)],
        [0],
        [1 / M]
    ])

    return A, B


def compute_lqr_gain(Q, R):
    """
    Solve the continuous-time Algebraic Riccati Equation (CARE)
    to find the optimal LQR gain matrix K.
    """
    A, B = linearize_system()

    # Solve CARE: A^T P + P A - P B R^-1 B^T P + Q = 0
    P = solve_continuous_are(A, B, Q, R)

    # Compute gain: K = R^-1 B^T P
    K = np.linalg.inv(R) @ B.T @ P

    return K


def simulate_lqr(K, y0, t_end=5.0, dt=0.01):
    """Simulate the pendulum with an LQR controller."""
    t_vals    = [0.0]
    y_vals    = [list(y0)]
    y_current = list(y0)
    t         = 0.0

    while t < t_end:
        # State vector
        x = np.array(y_current)

        # LQR control law: F = -K @ x
        F = float((-K @ x).item())

        # Clamp force
        F = np.clip(F, -20, 20)

        # Step forward
        sol = solve_ivp(
            fun=lambda t, y: pendulum(t, y, F),
            t_span=(t, t + dt),
            y0=y_current,
            method='RK45'
        )
        y_current = [sol.y[i][-1] for i in range(4)]
        t        += dt

        t_vals.append(t)
        y_vals.append(list(y_current))

        if abs(y_current[0]) > np.pi / 2:
            print(f"LQR: fell at t={t:.2f}s")
            break

    return np.array(t_vals), np.array(y_vals)

# 1. No control
t_open    = sol_open.t
theta_open = sol_open.y[0] * 180 / np.pi

# 2. PID
t_pid, y_pid   = simulate_pid(kp=50, ki=2, kd=8, y0=y0, t_end=3.0)
theta_pid      = y_pid[:, 0] * 180 / np.pi

# 3. LQR — tune Q and R here
Q = np.diag([10, 1, 1, 1])   # penalize angle most
R = np.array([[1]])           # control effort weight
K = compute_lqr_gain(Q, R)
print(f"LQR gain K = {K}")

t_lqr, y_lqr = simulate_lqr(K, y0, t_end=3.0)
theta_lqr     = y_lqr[:, 0] * 180 / np.pi

# ─── Plot ────────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(3, 1, figsize=(10, 12))

configs = [
    (axes[0], t_open, theta_open, '#A32D2D', 'No controller — pendulum falls freely'),
    (axes[1], t_pid,  theta_pid,  '#185FA5', 'PID controller (Kp=50, Ki=2, Kd=8)'),
    (axes[2], t_lqr,  theta_lqr,  '#3B6D11', 'LQR controller (optimal gains)'),
]


for ax, t, theta, color, title in configs:
    ax.plot(t, theta, color=color, linewidth=2)
    ax.axhline(0, color='gray', linestyle='--', linewidth=0.8)
    ax.set_title(title)
    ax.set_ylabel('Angle (degrees)')
    ax.set_xlabel('Time (s)')
    ax.set_ylim(-30, 90)
    ax.grid(True)

plt.tight_layout()
plt.savefig('simulation_result.png', dpi=150)
plt.show()
print("Plot saved.")