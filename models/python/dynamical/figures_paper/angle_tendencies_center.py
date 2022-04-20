import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import fmin, fsolve
from models import *
from fold2m import *

a1 = 7.0
a2 = 2.0
b = 72.5
sigma = 10.0
g1 = lambda u, v, rho: -a1*u + b*(1 - u)*np.exp(-sigma*(rho + 1.0)**2)
g2 = lambda u, v, rho: -a2*v + b*(1 - v)*np.exp(-sigma*(rho - 1.0)**2)

# Integrating model
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = a1
model.a2 = a2
model.sigma = sigma
model.epsilon = 0.005
t,X = model.integrate(T = 100000, q0 = [1.0,1.0, np.sqrt(0.25)])

ub = np.linspace(0, 1, 40)
fig, ax = plt.subplots(1,1)

plot_fold_projection( ax )

plt.plot(X[0,:], X[1,:], 'b')

D = np.zeros_like(ub)

for i in range(len(ub)):
    r1 = np.array([g1(ub[i], ub[i], np.sqrt(ub[i])), g2(ub[i], ub[i], np.sqrt(ub[i]))])
    r1p = 0.05*r1#/np.linalg.norm(r1)
    r2 = np.array([g1(ub[i], ub[i], -np.sqrt(ub[i])), g2(ub[i], ub[i], -np.sqrt(ub[i]))])
    r2p = 0.05*r2#/np.linalg.norm(r2)

    D[i] = np.dot(r1, r2)

    plt.arrow(ub[i], ub[i], r1p[0], r1p[1], width = 0.005, color = 'k')
    plt.arrow(ub[i], ub[i], r2p[0], r2p[1], width = 0.005, color = 'r')

psi = lambda u, a, b: 2.0*(a**2*u**2 - 2.0*a*b*u*(1 - u)*np.exp(-sigma*(u + 1.0))*np.cosh(2.0*sigma*np.sqrt(u)) + \
                    b**2*(1 - u)**2*np.exp(-2.0*sigma*(u + 1.0)))
a = a1
f = lambda u: psi(u, a, b)
pmin = fmin(f, 0.5)
plt.plot([pmin], [pmin], 'b*')
K = lambda rho: 2.0*(rho*np.cosh(2.0*sigma*rho) - np.sinh(2.0*sigma*rho))*np.exp(-sigma*(rho**2 + 1))
eta = lambda rho: b*K(rho)/(2.0*a*rho + b*K(rho))
ll = lambda u: 2.0*a/b*np.sqrt(u)
ff = lambda u: -K(np.sqrt(u))
gg = lambda x: (5.0*a/(6.0*np.sqrt(6)))*x**(3.0/2.0) - b*np.exp(-sigma*(x + 6.0)/6.0)*(\
            (2.0*x/np.sqrt(6) - 5.0*x**(3.0/2.0)/(3.0*np.sqrt(6)))*np.cosh(2.0*sigma*np.sqrt(x/6.0)) + \
            (x**2/9.0 - 2.0 + x)*np.sinh(2.0*sigma*np.sqrt(x/6.0)))
xr1 = fsolve(gg, 0.2)
xr2 = fsolve(gg, 0.8)
yr1 = np.sqrt(2.0*xr1**3/27.0)
yr2 = np.sqrt(2.0*xr2**3/27.0)
ur1 = (xr1 + yr1)/2.0
vr1 = (xr1 - yr1)/2.0
ur2 = (xr2 + yr2)/2.0
vr2 = (xr2 - yr2)/2.0
print(ur1)
print(ur2)
plt.plot([ur1], [ur1], 'b*', markersize = 10.0)
plt.plot([ur2], [ur2], 'g*', markersize = 10.0)

plt.figure()

plt.plot(ub, D, 'b')
plt.plot(ub, psi(ub, a, b), 'r--')
plt.plot([0, 1], [0, 0], 'k--')
plt.plot([pmin, pmin], [-30, 1], 'k--')

# plt.figure()
# bs = np.linspace(5, 90, 40)
# mins = np.zeros_like(bs)

# for i in range(len(bs)):
#     f = lambda u: psi(u, a, bs[i])
#     pmin = fmin(f, 0.7)
#     mins[i] = pmin
#     # plt.plot(ub, psi(ub, a, bs[i]))
#     # plt.plot([pmin, pmin], [-30, 1], 'k--')
# plt.plot(bs, mins)


plt.figure()

us = np.linspace(0, 1, 100)
plt.plot(us, us, 'k')
plt.plot(us, eta(np.sqrt(us)), 'r')
plt.plot([ur1], [ur1], 'r*')
plt.axis([0, 1, -5, 5])

plt.figure()

plt.plot(us, ll(us), 'k')
plt.plot(us, ff(us), 'r')
plt.plot([ur1], [ll(ur1)], 'r*')
plt.show()