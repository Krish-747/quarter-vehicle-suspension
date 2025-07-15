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

## Passive Suspension
Executing `Passive Suspension.py` simulates a basic suspension system with damping. The values can be modified in the *constant* section

## Active Suspension
Executing `Active Suspension.py` simulates a PID controlled active suspension system to maintain the set ride height. However the gains are not optimised and are just set randomly

## Comparison
To visualise the comparison between the Active and Passive Suspension, execute `Comparison.py` which shows the difference between the active and passive suspension on the same road disturbance.

![Comparison between Passive and Active Suspension](/Comparison%20Code.png)