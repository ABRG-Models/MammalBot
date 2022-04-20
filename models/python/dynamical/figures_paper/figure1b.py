import matplotlib.pyplot as plt  
import numpy as np 
from models import *


# Plotting the trajectory
a = 2.0
b = 65.0
sigma = 10.0

# Integrating model and plotting closed trajectory
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a
model.sigma = sigma
model.epsilon = 0.01

u0 = 0.75
t,X = model.integrate(T = 40000, q0 = [u0, u0, np.sqrt(u0)])
uav = np.mean(X[0,:])
vav = np.mean(X[1,:])
rav = np.mean(X[2,:])

fig, ax = plt.subplots(3,1)
ax[0].plot( t, X[0, :], 'b', linewidth = 2.0)
ax[0].plot( [0.0, t[-1]],[uav, uav], 'b--' )


ax[1].plot( t, X[1, :], 'r', linewidth = 2.0)
ax[1].plot( [0.0, t[-1]],[vav, vav], 'r--' )


ax[2].plot( t, X[2, :], 'k', linewidth = 2.0)

plt.show()