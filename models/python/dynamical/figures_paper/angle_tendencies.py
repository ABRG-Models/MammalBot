import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import fmin, fsolve
from models import *
from fold2m import *

a = 2.0
b = 60.0
sigma = 10.0
g1 = lambda u, v, rho: -a*u + b*(1 - u)*np.exp(-sigma*(rho + 1.0)**2)
g2 = lambda u, v, rho: -a*v + b*(1 - v)*np.exp(-sigma*(rho - 1.0)**2)

# Integrating model
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a
model.sigma = sigma
model.epsilon = 0.01

u0 = 0.0
v0 = 0.2
p = lambda v: (2.0 - np.sqrt(-(1.0/sigma)*np.log(a*v/(b*(1.0 - v)))))**2
t,X = model.integrate(T = 20000, q0 = [u0, v0, p(v0)])

ub = np.linspace(0, 1, 40)
fig, ax = plt.subplots(1,2)

plot_fold_projection( ax[0] )

ax[0].plot(X[0,:], X[1,:], 'b')
# for i in range(len(X[0,:])):
#     ax[0].plot(X[0,i], X[1,i], 'b.', markersize = 0.4)
#     ax[1].plot(t[i],X[0,i], 'b.', markersize = 1.0)
#     ax[1].plot(t[i],X[1,i], 'r.', markersize = 1.0)
#     ax[1].plot(t[i],X[2,i], 'k.', markersize = 1.0)
#     plt.pause(0.001)

D = np.zeros_like(ub)

for i in range(len(ub)):
    r1 = np.array([g1(ub[i], ub[i], np.sqrt(ub[i])), g2(ub[i], ub[i], np.sqrt(ub[i]))])
    r1p = 0.05*r1#/np.linalg.norm(r1)
    r2 = np.array([g1(ub[i], ub[i], -np.sqrt(ub[i])), g2(ub[i], ub[i], -np.sqrt(ub[i]))])
    r2p = 0.05*r2#/np.linalg.norm(r2)

    D[i] = np.dot(r1, r2)

    plt.arrow(ub[i], ub[i], r1p[0], r1p[1], width = 0.005, color = 'k')
    plt.arrow(ub[i], ub[i], r2p[0], r2p[1], width = 0.005, color = 'r')


# ll = lambda x: (5.0*a/(6.0*np.sqrt(6)))*x**(3.0/2.0)
# ff = lambda x: b*np.exp(-sigma*(x + 6.0)/6.0)*(\
#             (2.0*x/np.sqrt(6) - 5.0*x**(3.0/2.0)/(3.0*np.sqrt(6)))*np.cosh(2.0*sigma*np.sqrt(x/6.0)) + \
#             (x**2/9.0 - 2.0 + x)*np.sinh(2.0*sigma*np.sqrt(x/6.0)))
# L = lambda x, y, z: a*(x*z + y)/2.0
# F = lambda x, y, z: (b/2.0)*np.exp(-sigma*(z**2 + 1))*((2.0*z - x*z -y)*np.cosh(2.0*sigma*z) + (y*z - 2.0 + x)*np.sinh(2.0*sigma*z))
g1x = lambda x, y, z: -a*x + b*np.exp(-sigma*(z**2 + 1))*((2.0 - x)*np.cosh(2.0*sigma*z) + y*np.sinh(2.0*sigma*z))
g2x = lambda x, y, z: -a*y - b*np.exp(-sigma*(z**2 + 1))*((2.0 - x)*np.sinh(2.0*sigma*z) + y*np.cosh(2.0*sigma*z))
fx = lambda x, y, z: -z/2.0
fy = lambda x, y, z: -1.0/2.0
y = lambda x: -2.0*x**(3.0/2.0)/(3.0*np.sqrt(6.0))
z = lambda x: np.sqrt(x/6.0)

gg = lambda x: fx(x, y(x), z(x))*g1x(x, y(x), z(x)) + fy(x, y(x), z(x))*g2x(x, y(x), z(x))
xr1 = fsolve(gg, 0.5)
xr2 = fsolve(gg, 1.8)


yr1 = y(xr1)
yr2 = y(xr2)
print(xr1)
print(xr2)
print(yr1)
print(yr2)
print('----')
ur1 = (xr1 + yr1)/2.0
vr1 = (xr1 - yr1)/2.0
ur2 = (xr2 + yr2)/2.0
vr2 = (xr2 - yr2)/2.0
print(ur1)
print(ur2)
plt.plot([ur1], [vr1], 'b*', markersize = 10.0)
plt.plot([ur2], [vr2], 'g*', markersize = 10.0)

us = np.linspace(0, 2, 100)
plt.figure()

plt.plot(us, gg(us), 'k')
plt.plot([xr1], [gg(xr1)], 'r*')
plt.plot([xr2], [gg(xr2)], 'r*')
plt.show()