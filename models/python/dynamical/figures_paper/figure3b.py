# In figure 3 we analyze the time budget and explain the dominance boundary phenomenon
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import fmin, fsolve
from models import *
from fold2m import *

a = 2.0
b = 75.0
sigma = 10.0
g1 = lambda u, v, rho: -a*u + b*(1 - u)*np.exp(-sigma*(rho + 1.0)**2)
g2 = lambda u, v, rho: -a*v + b*(1 - v)*np.exp(-sigma*(rho - 1.0)**2)

# Integrating model
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a
model.sigma = sigma
model.epsilon = 0.01

u0 = 1.0
v0 = 1.0
t,X = model.integrate(T = 100000, q0 = [u0, v0, 1.0])

model.b1 = model.b2 = b
model.a1 = 3.8
model.a2 = a
model.sigma = sigma
model.epsilon = 0.01

u0 = 1.0
v0 = 1.0
t,Xd = model.integrate(T = 100000, q0 = [u0, v0, 1.0])

fig, ax = plt.subplots(2,1)

ax[0].plot(t, X[0,:], 'r', linewidth = 2.0)
ax[0].plot(t, X[1,:], 'r--', linewidth = 2.0)

ax[1].plot(t, Xd[0,:], 'b', linewidth = 2.0)
ax[1].plot(t, Xd[1,:], 'b--', linewidth = 2.0)

ax[0].set_xticks([0, t[-1]])
ax[0].set_yticks([0, 1])
ax[0].set_xticklabels([0,1], fontsize = 14.0)
ax[0].set_yticklabels([0,1], fontsize = 14.0)

ax[1].set_xticks([0, t[-1]])
ax[1].set_yticks([0, 1])
ax[1].set_xticklabels([0,1], fontsize = 14.0)
ax[1].set_yticklabels([0,1], fontsize = 14.0)
plt.show()
