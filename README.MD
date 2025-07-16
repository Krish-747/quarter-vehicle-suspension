# Quarter vehicle suspension system

This is a simulation of a quarter vehicles suspension as it is subjected to a random disturbance (smooth spline curve). The plot shows disturbance vs time (not to be confused with a vertical displacement vs *horizontal displacement*)

---
### Requirements
- Python 3.xx
- numpy
- scipy
- matplotlib

### Forces involved
- Spring Force
- Gravitational Force
- Damping Force
- Control Force from the PID Controller (Only in active suspension)

### Constants
These values in the *constant* can be changed to create differet simulations
- Mass [`mass`]
- Spring Constant [`spring_constant`]
- Damping Constant [`damping_constant`]
- Gravitational Constant [`g`]
- Spring Length [`spring_length`]
- Time [`total_time_ms`]
- Time Intervals (Increased value increases the *noisiness* of the road profile) [`time_intervals`]
- Road Height range [`height_range`]

These are only for the PID Controller
- Proportional Gain (Kp)
- Integral Gain (Ki)
- Derivative Gain (Kd)
---

## Passive Suspension
Executing `Passive Suspension.py` simulates a basic suspension system with damping. The values can be modified in the *constant* section

## Active Suspension
Executing `Active Suspension.py` simulates a PID controlled active suspension system to maintain the set ride height. The control force adjusts force based on the Proportional Error, Accumulated Error and Derivative Error. However the gains (Ki, Kp, Kd) are not optimised and are just set values. 

## Comparison
To visualise the comparison between the Active and Passive Suspension, execute `Comparison.py` which shows the difference between the active and passive suspension on the same road disturbance.

![Comparison between Passive and Active Suspension](/Comparison%20of%20Suspension.png)