# In figure 3 we analyze the time budget and explain the dominance boundary phenomenon
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import fmin, fsolve
from models import *
from fold2m import *

a1 = 2.0
a2 = 7.0
b = 65.0
sigma = 10.0
g1 = lambda u, v, rho: -a1*u + b*(1 - u)*np.exp(-sigma*(rho + 1.0)**2)
g2 = lambda u, v, rho: -a2*v + b*(1 - v)*np.exp(-sigma*(rho - 1.0)**2)

# Integrating model
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a1
model.sigma = sigma
model.epsilon = 0.01

u0 = 1.0
v0 = 1.0
t,X = model.integrate(T = 50000, q0 = [u0, v0, 1.0])

model.b1 = model.b2 = b
model.a1 = a1
model.a2 = a2
model.sigma = sigma
model.epsilon = 0.01

u0 = 1.0
v0 = 1.0
t,Xd = model.integrate(T = 50000, q0 = [u0, v0, 1.0])

fig, ax = plt.subplots(1,1)

plot_fold_projection( ax )

plt.plot(X[0,:], X[1,:], 'r', linewidth = 0.5)
plt.plot(Xd[0,:], Xd[1,:], 'b', linewidth = 2.0)

ax.set_xticks([0, 1])
ax.set_yticks([0, 1])
ax.set_xticklabels([0,1], fontsize = 14.0)
ax.set_yticklabels([0,1], fontsize = 14.0)
plt.show()
