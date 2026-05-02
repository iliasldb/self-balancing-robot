import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

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
t_end   = 3.0
t_eval  = np.linspace(t_start, t_end, 1000)

# Simulate with zero force (no controller)
F = 0.0
sol_open = solve_ivp(
    fun=lambda t, y: pendulum(t, y, F),
    t_span=(t_start, t_end),
    y0=y0,
    t_eval=t_eval,
    method='RK45'
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

# Run uncontrolled
t_open = sol_open.t
theta_open = sol_open.y[0] * 180 / np.pi   # convert to degrees

# Run with PID — tune these gains
t_pid, y_pid = simulate_pid(kp=50, ki=2, kd=8, y0=y0, t_end=5.0)
theta_pid = y_pid[:, 0] * 180 / np.pi

# Plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))

ax1.plot(t_open, theta_open, color='#A32D2D', linewidth=2)
ax1.axhline(0, color='gray', linestyle='--', linewidth=0.8)
ax1.set_title('No controller — pendulum falls freely')
ax1.set_ylabel('Angle (degrees)')
ax1.set_xlabel('Time (s)')
ax1.grid(True, alpha=0.3)

ax2.plot(t_pid, theta_pid, color='#185FA5', linewidth=2)
ax2.axhline(0, color='gray', linestyle='--', linewidth=0.8)
ax2.set_title('PID controller — pendulum stabilizes')
ax2.set_ylabel('Angle (degrees)')
ax2.set_xlabel('Time (s)')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('simulation_result.png', dpi=150)
plt.show()
print("Plot saved as simulation_result.png")