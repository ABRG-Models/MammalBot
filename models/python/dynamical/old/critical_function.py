import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
a1 = lambda a2, rho: 2*(rho*(rho-1)*(2*rho -1) + a2*rho/2)/(rho -1)
delta = 0.025
a2 = np.arange(0, 1, delta)
rho = np.arange(0.0, 0.8, delta)
A,R = np.meshgrid(a2, rho)
Z = a1(A, R)

surf = ax.plot_surface(A, R, Z, cmap = cm.coolwarm)

ax.set_zlim(-1, 1)

plt.show()