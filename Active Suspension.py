import numpy as np
import random
from scipy.interpolate import make_interp_spline
import matplotlib.pyplot as plt

# * Constants
mass = 5.0              # kg
spring_constant = 50.0  # N/m
damping_constant = 50.0 # Ns/m
g = 10.0                # m/s²
spring_length = 10.0    # m

# * PID Controller
Kp = 150
Ki = 50
Kd = 50

sum_error = 0
prev_error = 0

# * Time 
total_time_ms = 5000
time_ms = np.linspace(0, total_time_ms, 1000)
time_s = time_ms / 1000.0
dt = time_s[1] - time_s[0]

# * Road
road_x = np.linspace(0, total_time_ms, 10)
road_y = [0] + [random.uniform(-2.5, 2.5) for _ in range(9)]
road_spline = make_interp_spline(road_x, road_y)
road_y = road_spline(time_ms)

# * Initial conditions
equilibrium_compression = (mass * g) / spring_constant
initial_y = road_y[0] + spring_length - equilibrium_compression
y = initial_y
v = 0
positions = [y]

# * Loop
for i in range(1, len(time_s)):
    y_road = road_y[i]

    spring_extension = spring_length - (y - y_road)
    F_spring = spring_constant * spring_extension
    F_gravity = mass * g
    F_damping = damping_constant * v
    
    # ! PID Part
    error = initial_y - y
    sum_error += error * dt
    d_error = (error - prev_error) / dt
    prev_error = error
    
    F_control = (Kp * error) + (Ki * sum_error) + (Kd * d_error)

    F_net = F_spring - F_gravity - F_damping + F_control
    a = F_net / mass

    v += a * dt
    y += v * dt

    positions.append(y)

# * Plot
plt.figure(figsize=(12, 6))
plt.plot(time_ms, road_y, label="Road", color='black', linestyle='--', linewidth=2)
plt.plot(time_ms, positions, label="Active Suspension", color='orange')
plt.axhline(initial_y, color='gray', linestyle=':', label="Height")

plt.xlabel("Time (ms)")
plt.xlim(0, 5000)

plt.ylabel("Vertical Position (m)")
plt.ylim(-4, 15)

plt.title("Active Suspension")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()