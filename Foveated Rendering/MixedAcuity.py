import numpy as np
import matplotlib.pyplot as plt

N_max = 64
N_min = 4

# Eccentricity range
e = np.linspace(0.0, 30.0, 600)

# Photoreceptor-limited MAR (linear)
a = 0.02 # MAR(0)
b = 0.01
MAR_photo = a + b * e

# Ganglion-limited MAR (steeper growth)
c = 0.015
d = 0.08
MAR_ganglion = a + c * np.log(1.0 + d * e)

# Mixed MAR
MAR_mixed = np.maximum(MAR_photo, MAR_ganglion)

# Sample size from mixed MAR
N = N_max * (MAR_mixed[0] / MAR_mixed)**2
N = np.clip(N, N_min, N_max)

plt.figure()
plt.plot(e, N)
plt.xlabel("Eccentricity e (degrees)")
plt.ylabel("Sample size N(e) (samples per pixel)")
plt.title("Mixed Acuity Model Sample Falloff")
plt.grid(True)
plt.xticks(np.arange(0, 31, 2.5))
plt.show()