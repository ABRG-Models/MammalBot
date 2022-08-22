import numpy as np
import matplotlib.pyplot as plt

rho1 = -1.0
rho2 = 1.0
f = lambda u, v, rho: (rho - rho1)**2*(rho - rho2)**2 + (1 - u)*(rho - rho1)**2 + (1 - v)*(rho - rho2)**2

rho = np.linspace(-1.4,1.4,100)

fig, ax = plt.subplots(2, 1)
u = 1.0
v = 1.0
ax[0].plot(rho, f(u, v, rho), linewidth = 3.0)
u = 0.5
v = 1.0
ax[1].plot(rho, f(u, v, rho), linewidth = 3.0)
ax[1].axis([-1.4, 1.4, 0, 1])
plt.show()