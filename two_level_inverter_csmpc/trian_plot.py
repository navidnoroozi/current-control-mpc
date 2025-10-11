import matplotlib.pyplot as plt
import numpy as np

def _triangle_01(fc,t):
        """Triangle in [0,1] with period 1/fc."""
        Tc = 1.0 / fc
        tau = t % Tc  # t mod Tc
        half = 0.5 * Tc # to determine if t is the first half or second half
        k = t // Tc   # the floor division // rounds the result down to the nearest whole number
        if tau <= half:
            return (t - k*Tc) / half
        else:
            return -(t - (k+1)*Tc) / half
fc = 2.0
Tc = 1/fc
c = []

t = np.arange(0, 1.01, 0.01)
c = [_triangle_01(fc,t0) for t0 in t]

plt.plot(t, c, label='c(t)')
plt.show()