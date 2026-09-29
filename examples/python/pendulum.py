"""A passive pendulum experiment using only physical simulation APIs."""
import math

import csim

model = csim.PendulumModel(length=1, mass=1, timestep=0.01)
data = csim.make_data(model, angle=0.7)
initial = csim.get_state(model, data)
max_energy_error = 0.0
for _ in range(2000):
    csim.step(model, data)
    state = csim.get_state(model, data)
    max_energy_error = max(max_energy_error, abs(state['energy']/initial['energy'] - 1))

print(f"time: {state['time']:.3f} s")
print(f"angle: {state['angle']:.6f} rad")
print(f"position_W: {state['position_W']} m")
print(f"tension: {state['tension']:.6f} N")
print(f"max relative energy error: {max_energy_error:.3e}")
print(f"cable length error: {abs(math.dist(state['position_W'], model.pivot_W)-model.length):.3e} m")
