import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin, fsolve
from models import *
from fold2m import *
a1 = 2.0
a2 = 2.5
b = 75.5
sigma = 10.0

model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = a1
model.a2 = a2
model.sigma = sigma
model.epsilon = 0.01

u0 = 1.0
v0 = 1.0
t,X = model.integrate(T = 50000, q0 = [u0, v0, -1.0])

g1 = lambda u, v, rho: -a1*u + b*(1 - u)*np.exp(-sigma*(rho + 1.0)**2)
g2 = lambda u, v, rho: -a2*v + b*(1 - v)*np.exp(-sigma*(rho - 1.0)**2)
fu = lambda u, v, rho: -(rho + 1.0)/2.0
fv = lambda u, v, rho: -(rho - 1.0)/2.0

def u(v, s = 1.0):
    f = lambda u: s*np.sqrt(2.0*(u + v)**3/27.0) + v - u
    # f = lambda u: 2.0*(u + v)**3 - 27.0*(v - u)**2
    ur = fsolve(f, 0.0)
    return ur[0]

def v(u, s = 1.0):
    f = lambda v: s*np.sqrt(2.0*(u + v)**3/27.0) + v - u
    # f = lambda u: 2.0*(u + v)**3 - 27.0*(v - u)**2
    vr = fsolve(f, 0.0)
    return vr[0]

# First branch (left)
vs = np.linspace(0, 1, 20)
s = -1.0
sc = 0.05
tc1 = np.zeros_like(vs)
ff1 = np.zeros_like(vs)

fig, ax = plt.subplots(1,1)

for i in range(len(vs)):
    uv = u(vs[i], s)
    rho = lambda u, v: -1.0*s*np.sqrt((u + v)/6.0)
    gg = lambda v: fu(uv, v, rho(uv, v))*g1(uv, v, rho(uv, v)) + fv(uv, v, rho(uv, v))*g2(uv, v, rho(uv, v))
    tc1[i] = gg(vs[i])
    ff1[i] = uv

    plt.arrow(uv, vs[i], sc*fu(uv, vs[i], rho(uv, vs[i])), sc*fv(uv, vs[i], rho(uv, vs[i])), width = 0.005, color = 'k' )
    plt.arrow(uv, vs[i], sc*g1(uv, vs[i], rho(uv, vs[i])), sc*g2(uv, vs[i], rho(uv, vs[i])), width = 0.005, color = 'r' )

# Second branch (right)
us = np.linspace(0, 1, 20)
s = 1.0
tc2 = np.zeros_like(vs)
ff2 = np.zeros_like(vs)

for i in range(len(vs)):
    vu = v(us[i], s)
    rho = lambda u, v: -1.0*s*np.sqrt((u + v)/6.0)
    gg = lambda u: fu(u, vu, rho(u, vu))*g1(u, vu, rho(u, vu)) + fv(u, vu, rho(u, vu))*g2(u, vu, rho(u, vu))
    tc2[i] = gg(us[i])
    ff2[i] = vu

    plt.arrow(us[i], vu, sc*fu(us[i], vu, rho(us[i], vu)), sc*fv(us[i], vu, rho(us[i], vu)), width = 0.005, color = 'k' )
    plt.arrow(us[i], vu, sc*g1(us[i], vu, rho(us[i], vu)), sc*g2(us[i], vu, rho(us[i], vu)), width = 0.005, color = 'r' )

# ax.plot(ff1, vs, 'b--', linewidth = 3.0)
# ax.plot(us, ff2, 'r--', linewidth = 3.0)
plot_fold_projection(ax)
ax.axis([0,1,0,1])
plt.plot(X[0,:], X[1,:], 'b', linewidth = 2.0)

fig, ax = plt.subplots(1,1)
plt.plot(vs, tc1, 'b')
plt.plot(us, tc2, 'r')
plt.plot([0, 1], [0, 0], 'k--')



ax.set_xticks([0, 1])
ax.set_yticks([0, 1])
ax.set_xticklabels([0,1], fontsize = 14.0)
ax.set_yticklabels([0,1], fontsize = 14.0)



plt.show()