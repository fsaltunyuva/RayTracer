import numpy as np
import matplotlib.pyplot as plt

N_max = 64
N_min = 4

e0 = 1.0  # in degrees
e = np.linspace(0.0, 20.0, 600)

# Log Acuity Falloff
N = N_max * (e0 / (e + e0))**2
N = np.clip(N, N_min, N_max)

plt.figure()
plt.plot(e, N)
plt.xlabel("Eccentricity e (degrees)")
plt.ylabel("Sample size N(e) (samples per pixel)")
plt.title("Log Acuity Model Sample Falloff")
plt.grid(True)
plt.show()