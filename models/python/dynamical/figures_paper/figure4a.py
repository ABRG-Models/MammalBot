# Dominance - dominance boundary
import numpy as np
import matplotlib.pyplot as plt
from models import *
from fold2m import *

# Plotting the fold
a = 2.0
b = 50.0
sigma = 10.0

# Integrating model
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a
model.sigma = sigma

t_interrupt = 200

def Fi(t):
    if( t > t_interrupt and t < t_interrupt+10 ):
        return -1
    return 0.0

u0 = 0.2
v0 = 0.1
t,X = model.integrate(T = 50000, q0 = [u0, v0, -1.0], input_u = Fi)
  
fig, ax = plt.subplots(1,1)

plot_fold_projection( ax )

plt.plot(X[0,:], X[1,:], 'b', linewidth = 2.0)

plt.figure()
plt.plot(1-X[0, :], 1- X[1, :])
plt.show()