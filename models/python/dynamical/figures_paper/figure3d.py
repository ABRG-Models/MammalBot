# Determine time budget
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin, fsolve
from models import *
from fold2m import *

# Determining allowed range
ass = np.linspace(2.0, 7.0, 50)

a1 = 2.0
b = 75.5
sigma = 10.0

def u(v, s = 1.0):
    f = lambda u: s*np.sqrt(2.0*(u + v)**3/27.0) + v - u
    # f = lambda u: 2.0*(u + v)**3 - 27.0*(v - u)**2
    ur = fsolve(f, 0.0)
    return ur[0]

lima = None

for k in range(len(ass)):
    a2 = ass[k]
    g1 = lambda u, v, rho: -a1*u + b*(1 - u)*np.exp(-sigma*(rho + 1.0)**2)
    g2 = lambda u, v, rho: -a2*v + b*(1 - v)*np.exp(-sigma*(rho - 1.0)**2)
    fu = lambda u, v, rho: -(rho + 1.0)/2.0
    fv = lambda u, v, rho: -(rho - 1.0)/2.0

    # First branch (left)
    vs = np.linspace(0, 1, 100)
    s = -1.0
    tc1 = np.zeros_like(vs)

    for i in range(len(vs)):
        uv = u(vs[i], s)
        rho = lambda u, v: -1.0*s*np.sqrt((u + v)/6.0)
        gg = lambda v: fu(uv, v, rho(uv, v))*g1(uv, v, rho(uv, v)) + fv(uv, v, rho(uv, v))*g2(uv, v, rho(uv, v))
        tc1[i] = gg(vs[i])

    # crossings = np.where(np.diff(np.sign(tc1)))
    # print(crossings)
    # if crossings[0][-1] > 2.0*len(vs)/3.0:
    #     lima = ass[k]
    #     break
    if np.trapz(tc1) < 0:
        lima = ass[k]
        break


# Estimating time budget


model = TwoMotivations()
ass = np.linspace(a1, lima, 10)

time1 = np.zeros_like(ass)
time2 = np.zeros_like(ass)

for i in range(len(ass)):
    model.b1 = model.b2 = b
    model.a1 = a1
    model.a2 = ass[i]
    model.sigma = sigma
    model.epsilon = 0.01

    u0 = 0.3
    v0 = 0.3
    t,X = model.integrate(T = 100000, q0 = [u0, v0, 1.0])

    time1[i] = len(np.where(X[2,:] > 0)[0])/len(X[2,:])
    time2[i] = len(np.where(X[2,:] < -0)[0])/len(X[2,:])


plt.stackplot(ass,time1, time2, labels=['Activity 1','Activity 2'])
# plt.plot(time1)
# plt.plot(time2)
plt.axis([a1, lima, 0, 1])
plt.show()