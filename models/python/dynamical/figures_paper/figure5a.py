# Recovering from death
import numpy as np
import matplotlib.pyplot as plt
from models import *
from fold2m import *

# Plotting the fold
a = 2.0
b = 20.0
sigma = 10.0

# Integrating model
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a
model.sigma = sigma

def Fi(t):
    if( t > 40 and t < 240 ):
        return 100.0
    return 0.0

u0 = 0.2
t,X = model.integrate(T = 100000, q0 = [u0, u0, np.sqrt(u0)], input_u = Fi)

fig, ax = plt.subplots(1,1)

plot_fold_projection( ax )

plt.plot(X[0,:], X[1,:], 'b', linewidth = 2.0)

plt.figure()
plt.plot(t, X[0, :])
plt.plot(t, X[1, :])
plt.show()