import numpy as np
import matplotlib.pyplot as plt

N_max = 64
N_min = 4

a = 0.02 # MAR(0)
b = 0.01 # linear MAR slope

# Eccentricity range
e_min, e_max = 0.0, 30.0
e = np.linspace(e_min, e_max, 600)

# Linear Acuity Falloff
N = N_max * (a / (a + b * e))
N = np.clip(N, N_min, N_max)

plt.figure()
plt.plot(e, N)
plt.xlabel("Eccentricity e (degrees)")
plt.ylabel("Sample size S(e) (samples per pixel)")
plt.title("Linear Acuity Model Sample Falloff")
plt.grid(True)

plt.xticks(np.arange(e_min, e_max + 1e-6, 2.5))

plt.show()