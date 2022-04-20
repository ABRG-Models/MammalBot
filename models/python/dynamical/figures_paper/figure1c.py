import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from models import *
from fold2m import *

a = 2.0
b = 65.0
sigma = 10.0

u0 = 1.0
v0 = 1.0
x0 = u0+v0
y0 = u0-v0
# Integrating original model
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a
# t,Xo = model.integrate(T = 22500, q0 = [u0, v0, 1.0])
t,Xo = model.integrate(T = 50500, q0 = [u0, v0, 1.0])

# Integrating original model
model = TwoMotivationsRotated()
model.b = b
model.a = a
# t,Xr = model.integrate(T = 22500, x0 = [x0, y0, 1.0])
t,Xr = model.integrate(T = 50500, x0 = [x0, y0, 1.0])

fig, ax = plt.subplots(1, 1)

plot_fold_projection( ax )
ax.plot(Xo[0,0:15500],Xo[1,0:15500], 'b', linewidth = 2.0)
ax.plot(Xo[0,15500:-1],Xo[1,15500:-1], 'r', linewidth = 2.0)
k = np.linspace(0, 2.0, 10)
us = np.linspace(0, 1, 10)

for i in range(len(k)):
    ax.plot(us, k[i] - us, 'k--', linewidth = 0.5)

ax.axis([0, 1, 0, 1])
ax.set_yticks([0, 1])
ax.set_yticklabels([0, 1], fontsize=14)
ax.set_xticks([0, 1])
ax.set_xticklabels([0, 1], fontsize=14)

fig, ax = plt.subplots(1, 1)
plot_fold_projection_rotated( ax )
ax.plot(Xr[0,0:15500],Xr[1,0:15500], 'b', linewidth = 2.0)
ax.plot(Xr[0,15500:-1],Xr[1,15500:-1], 'r', linewidth = 2.0)
us = np.linspace(0, 1, 10)

for i in range(len(k)):
    ax.plot([k[i], k[i]], [-1, 1], 'k--', linewidth = 0.5)

ax.set_yticks([-1, 0, 1])
ax.set_yticklabels([-1,0, 1], fontsize=14)
ax.set_xticks([0, 2])
ax.set_xticklabels([0, 2], fontsize=14)

ax.axis([0, 2, -1, 1])

plt.figure()
plt.plot(t, 1/Xr[0,:], 'g', linewidth = 2.0)
plt.plot(t, Xo[0, :], 'b', linewidth = 0.3)
plt.plot(t, Xo[1, :], 'r', linewidth = 0.3)
plt.xticks([])
plt.yticks([])
plt.show()