import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from models import *
from fold2m import *

# Integrating the model
model = TwoMotivations()
model.b1 = model.b2 = 97.0
# model.b1 = model.b2 = 25.0
model.a1 = 2.0
model.a2 = 2.0
t,X = model.integrate(T = 100000, q0 = [0.3, 0.0, 1.0])

model.b1 = model.b2 = 10.0
# model.b1 = model.b2 = 25.0
model.a1 = 2.0
model.a2 = 2.0
t,X2 = model.integrate(T = 100000, q0 = [0.3, 0.0, 1.0])

a = model.a1
b = model.b1
sigma = model.sigma
r = lambda u, v: np.sqrt((u + v)/6.0)
# du = lambda u,v: -a*u + b*(1 - u)*np.exp(-sigma*(r(u,v) + 1.0))
# dv = lambda u, v: -a*u + b*(1 - u)*np.exp(-sigma*(r(u,v) - 1.0))


## Plotting the trayectory on the fold
fig, ax = plt.subplots(1,2)
u = np.linspace(0, 1, 20)
v = np.linspace(0, 1, 20)
U, V = np.meshgrid(u, v)

plot_fold_projection( ax[0] )
ax[0].plot(X[0,:], X[1,:])
ax[0].plot(X2[0,:], X2[1,:], 'r')
# ax[0].plot(u, u, 'k')
du = lambda u,v: -(a + b)*u + b
dv = lambda u,v: -a*v + b*(1-v)*np.exp(-4.0*sigma)
DU = du(U,V)
DV = dv(U,V)
ax[0].quiver(U, V, DU, DV)
ax[0].set_aspect('equal', 'box')

plot_fold_projection( ax[1] )
ax[1].plot(X[0,:], X[1,:])
ax[1].plot(X2[0,:], X2[1,:], 'r')
# ax[1].plot(u, u, 'k')
du = lambda u,v: -a*u + b*(1-u)*np.exp(-4.0*sigma)
dv = lambda u,v: -(a + b)*v + b
DU = du(U,V)
DV = dv(U,V)
ax[1].quiver(U, V, DU, DV)
ax[1].set_aspect('equal', 'box')
plt.show()