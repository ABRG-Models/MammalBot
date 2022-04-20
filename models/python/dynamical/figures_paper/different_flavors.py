import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from models import *
from fold2m import *

a = 2.0
b = 25.0
sigma = 10.0

u0 = 1.0
v0 = 1.0
x0 = u0+v0
y0 = u0-v0
# Integrating original model
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a
t,Xo = model.integrate(T = 100000, q0 = [u0, v0, 1.0])

# Integrating original model
model = TwoMotivationsRotated()
model.b = b
model.a = a
t,Xr = model.integrate(T = 100000, x0 = [x0, y0, 1.0])

# Integrating original model

fig, ax = plt.subplots(1, 2)

ax[0].plot(t, Xo[0,:], 'r')
ax[0].plot(t, Xo[1,:], 'b')



ax[1].plot(t, Xr[0,:], 'r--')
ax[1].plot(t, Xr[1,:], 'b--')

fig, ax = plt.subplots(1, 2)

plot_fold_projection( ax[0] )
ax[0].plot(Xo[0,:],Xo[1,:], 'b')


plot_fold_projection_rotated( ax[1] )
ax[1].plot(Xr[0,:], Xr[1,:], 'b--')

plt.show()