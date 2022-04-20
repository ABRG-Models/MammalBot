import numpy as np
import matplotlib.pyplot as plt

# testing the shape of the kernels for location
m = 640
sigma = m/3.0
gl = lambda x: np.exp(-((x - 0)**2)/(2*sigma**2))
gc = lambda x: np.exp(-((-x + m)**2)/(2*sigma**2))

x = np.linspace(0, m, 100)
plt.plot(x,gl(x))
plt.plot(x,gc(x))
plt.show()