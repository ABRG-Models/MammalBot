import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from models import *
from fold2m import *

a = 2.0
b = 35.0
sigma = 10.0
g1 = lambda u, v, rho: -a*u + b*(1 - u)*np.exp(-sigma*(rho + 1.0)**2)
g2 = lambda u, v, rho: -a*v + b*(1 - v)*np.exp(-sigma*(rho - 1.0)**2)

# Integrating model
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a
t,X = model.integrate(T = 100000, q0 = [1.0, 1.0, 1.0])

ub = np.linspace(0, 1, 10)
fig, ax = plt.subplots(1,1)

plot_fold_projection( ax )

plt.plot(X[0,:], X[1,:], 'b')

D = np.zeros_like(ub)

for i in range(len(ub)):
    r1 = np.array([g1(ub[i], ub[i], np.sqrt(ub[i])), g2(ub[i], ub[i], np.sqrt(ub[i]))])
    r1 = 0.1*r1/np.linalg.norm(r1)
    r2 = np.array([g1(ub[i], ub[i], -np.sqrt(ub[i])), g2(ub[i], ub[i], -np.sqrt(ub[i]))])
    r2 = 0.1*r2/np.linalg.norm(r2)

    D[i] = np.dot(r1, r2)/(np.linalg.norm(r1)*np.linalg.norm(r2))

    plt.arrow(ub[i], ub[i], r1[0], r1[1], width = 0.005, color = 'k')
    plt.arrow(ub[i], ub[i], r2[0], r2[1], width = 0.005, color = 'r')

print(a/b)

def u_proc(u):
    r1 = np.array([g1(u, u, np.sqrt(u)), g2(u, u, np.sqrt(u))])
    r1 = 0.1*r1/np.linalg.norm(r1)
    r2 = np.array([g1(u, u, -np.sqrt(u)), g2(u, u, -np.sqrt(u))])
    r2 = 0.1*r2/np.linalg.norm(r2)
    p1 = np.array([u + r1[0],u + r1[1]])
    p2 = np.array([p1[0] - 0.1*u, p1[1] - 0.1*u])
    p3 = np.array([p2[0] + 2.0*r2[0],p2[1] + 2.0*r2[1]])
    p4 = np.array([p3[0] - 0.1*u, p3[1] - 0.1*u])
    p5 = np.array([p4[0] + r1[0], p4[1] + r1[1]])

    return p5

u0 = 0.9
plt.plot(u0, u0, 'k.', markersize = 10.0)

for i in range(1000):
    u1 = u_proc(u0)
    plt.plot(u1[0], u1[1], 'k.', markersize = 10.0)
    u0 = u1[0]

plt.plot(u1[0], u1[1], 'r.', markersize = 10.0)

plt.figure()
plt.plot(ub, D)
plt.plot([0, 1], [0, 0], 'k--')
plt.show()