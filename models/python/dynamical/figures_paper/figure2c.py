import numpy as np
import matplotlib.pyplot as plt
from models import *
from fold2m import *
from solver import Solver

s = Solver()
a = 2.0
b = 20.0
sigma = 10.0
u_rho = lambda rho: rho**2
K = lambda rho: np.exp(-sigma*(rho**2 + 1))*np.cosh(2.0*sigma*rho)
u = lambda rho: (b*K(rho))/(a + b*K(rho))

epsilon = 0.01
g1 = lambda u, rho: -a*u + b*(1.0 - u)*K(rho)
f = lambda u, rho: -4.0*(rho**3 - u*rho)
F = lambda t, x: np.array([epsilon*g1(x[0], x[1]),
                           f(x[0], x[1])])

T = 200000
u0 = 0.3
t, X = s.integrate(F, T, [u0, -0.01] )

rho = np.linspace(-1, 1, 100)

plt.plot(rho, u_rho(rho), linewidth = 2.5, color = (93.0/255.0, 139.0/255.0, 244.0/255.0))
plt.plot(rho, u(rho), linewidth = 2.5, color = (0.8, 0.3, 0.4))

x,y = np.meshgrid(np.linspace(0,1,10),np.linspace(-1,1,10))
plt.quiver(y, x, f(x, y), g1(x, y), color = (0.5, 0.5, 0.5))

plt.plot(X[1,:],X[0,:], 'k', linewidth = 2.0)
plt.plot(X[1,0],X[0,0], 'k.', markersize = 10.0)

plt.show()