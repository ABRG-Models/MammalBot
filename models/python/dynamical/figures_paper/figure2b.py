# In this figure we plot the poincare section fo different values of b
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


def get_stable_cycle( b ):
    model.b1 = model.b2 = b
    model.a1 = model.a2 = a
    model.sigma = sigma
    u0 = 1.0
    t,X = model.integrate(T = 80000, q0 = [u0, u0, np.sqrt(u0)])
    u = X[0, :]
    v = X[1, :]
    p = np.array([])

    for i in range(len(u)-1):
        
        if ((u[i] - v[i])*(u[i+1] - v[i+1]) <= 0):
            p = np.append(p,u[i+1])
    return p[-1]

def get_unstable_cycle(b, p_stable):
    model.b1 = model.b2 = b
    model.a1 = model.a2 = a
    model.sigma = sigma

    du = 0.01
    p_uns = 0.0
    u0 = 0.0

    while( np.abs(p_uns-p_stable) > 0.01 ):
        u0 = u0 + du
        t,X = model.integrate(T = 80000, q0 = [u0, u0, np.sqrt(u0)])
        u = X[0, :]
        v = X[1, :]
        p = np.array([])

        for i in range(len(u)-1):
            
            if ((u[i] - v[i])*(u[i+1] - v[i+1]) <= 0):
                p = np.append(p,u[i+1])
        p_uns = p[-1]

    return u0

def get_bifurcation_diagram( bs ):
    
    stables = np.array([])
    unstables = np.array([])

    for i in range(len(bs)):
        p_s = get_stable_cycle(bs[i])
        p_u = get_unstable_cycle(bs[i], p_s)

        if(p_s > 0.01):
            stables = np.append(stables, p_s)
            unstables = np.append(unstables, p_u)

    return stables, unstables

bs = np.linspace(100.0, 10.0, 40)
stables, unstables = get_bifurcation_diagram(bs)
print(stables)
plt.plot(bs[0:len(stables)], stables, 'k', linewidth = 2.0)
plt.plot(bs[0:len(stables)], unstables, 'r', linewidth = 2.0)

plt.show()

