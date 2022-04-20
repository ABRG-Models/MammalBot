import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from models import *
from fold2m import *

# Plotting the fold
a = 2.0
b = 45.0
sigma = 10.0

# Integrating model
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a
model.sigma = sigma

u0 = 1.0
t,X = model.integrate(T = 20000, q0 = [u0, u0, np.sqrt(u0)])

fig, ax = plt.subplots(1,1)

plot_fold_projection( ax )

# Plotting the two trajectories
u0 = 1.0
t,X = model.integrate(T = 200000, q0 = [u0, u0, np.sqrt(u0)])
ax.plot(X[0,:], X[1,:], 'b--', linewidth = 0.3)
u0 = 0.223
t,X = model.integrate(T = 200000, q0 = [u0, u0, np.sqrt(u0)])
ax.plot(X[0,:], X[1,:], 'r--', linewidth = 0.3)

u0 = 0.24
t,X = model.integrate(T = 200000, q0 = [u0, u0, np.sqrt(u0)])
ax.plot(X[0,:], X[1,:], 'b--', linewidth = 0.3)
# Plotting the boundaries


# Marking death
plt.plot([0], [0], 'k.', markersize = 25.0)

# Plotting limbo


ax.set_xticks([0, 1])
ax.set_yticks([0, 1])

plt.show()