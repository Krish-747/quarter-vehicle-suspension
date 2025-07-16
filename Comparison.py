import numpy as np
import random
from scipy.interpolate import make_interp_spline
import matplotlib.pyplot as plt

# * Constants
mass = 5.0             # kg
spring_constant = 50.0  # N/m
damping_constant = 50.0 # Ns/m
g = 10.0                # m/s²
spring_length = 20.0    # m

# * Road Profile
time_intervals = 100
height_range = (-10, 10)

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
dt = 0.01

# * Road
road_x = np.linspace(0, total_time_ms, time_intervals)
road_y = [0] + [random.uniform(height_range[0], height_range[1]) for _ in range(time_intervals-1)]
road_spline = make_interp_spline(road_x, road_y)
road_y = road_spline(time_ms)

# * Initial conditions
equilibrium_compression = (mass * g) / spring_constant
initial_y = road_y[0] + spring_length - equilibrium_compression

y = initial_y
v = 0
positions = [y]

y_passive = initial_y
v_passive = 0
passive_position = [y_passive]

# * Active Loop
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
    
# * Passive Loop
for i in range(1, len(time_s)):
    y_road = road_y[i]

    spring_extension = spring_length - (y_passive - y_road)
    F_spring = spring_constant * spring_extension
    F_gravity = mass * g
    F_damping = damping_constant * v_passive

    F_net = F_spring - F_gravity - F_damping
    a = F_net / mass

    v_passive += a * dt
    y_passive += v_passive * dt

    passive_position.append(y_passive)

# * Plot
plt.figure(figsize=(12, 6))
plt.plot(time_ms, road_y, label="Road", color='black', linestyle='--', linewidth=2)
plt.plot(time_ms, positions, label="Active Suspension", color='blue')
plt.plot(time_ms, passive_position, label="Passive Suspension", color='red')
plt.axhline(initial_y, color='gray', linestyle=':', label="Height")

plt.xlabel("Time (ms)")
plt.xlim(0, total_time_ms)

plt.ylabel("Vertical Position (m)")
plt.ylim(height_range[0]-5, height_range[1]+initial_y)
plt.yticks(np.arange(height_range[0], height_range[1]+initial_y, step=10))

plt.title("Comparision between Active and Passive Suspension")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()