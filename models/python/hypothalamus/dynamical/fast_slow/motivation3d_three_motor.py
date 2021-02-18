from mayavi import mlab
import matplotlib.pyplot as plt  

import numpy as np 

T = 550000
h = 0.001
a = 0.5
b = 0.5
x1 = -1.0
x2 = 1.0
x3 = 2.0
epsilon = 0.1
c1 = 0.3
c2 = 0.1
d1 = 3.0
d2 = 3.0
eta = lambda rho, c: np.exp(-(rho)**2/(2*c**2))
# xi = lambda x : 0.5 - 1.0/(1.0 + np.exp(-x))
xi = lambda x : x/np.max([0.0001, np.abs(x)])
# xi = lambda x: x
u = np.zeros(T)
v = np.zeros(T)
w = np.zeros(T)
rho = np.zeros(T)
x = np.zeros(T)
time = np.zeros(T)
# Maps
f = lambda rho, u, v, w, x: -(rho**3 - rho)*(3.0*rho**2 - 1) - u*(rho + 1) - v*rho - w*(rho - 1)
g1 = lambda rho, u, v, w, x: epsilon**2*(a*(1.0 - u) - d1*eta(x - x1, c2))
g2 = lambda rho, u, v, w, x: epsilon**2*(b*(1.0 - v) - d2*eta(x - x2, c2))
g3 = lambda rho, u, v, w,x: epsilon**2*(b*(1.0 - w) - d2*eta(x - x3, c2))
fm = lambda rho, u, v, w, x: epsilon*(-eta(rho + 1, c1)*xi(u)*xi(x - x1) - eta(rho, c1)*xi(v)*xi(x - x2) - eta(rho-1, c1)*xi(w)*xi(x - x3))
# Initial conditions
u[0] = 0.0
v[0] = 0.0
w[0] = 0.1
rho[0] = -0.9
x[0] = 0.0

for i in range(T-1):
    rho[i+1] = rho[i] + h*f(rho[i], u[i], v[i], w[i], x[i])
    u[i+1] = u[i] + h*g1(rho[i], u[i], v[i], w[i], x[i])
    v[i+1] = v[i] + h*g2(rho[i], u[i], v[i], w[i], x[i])
    w[i+1] = w[i] + h*g3(rho[i], u[i], v[i], w[i], x[i])
    x[i+1] = x[i] + h*fm(rho[i], u[i], v[i], w[i], x[i])
    time[i+1] = time[i] + h

# ux, vx, rhox = np.mgrid[-2:2:50j, -2:2:50j, -1.5:1.5:50j]
# values = -(rhox**3 - rhox)*(3.0*rhox**2 - 1) - ux*(rhox + 1) - vx*rhox - w*(rhox - 1)
# mlab.contour3d(ux, vx, rhox, values, contours=[0], opacity = 0.5)
# mlab.axes()
# mlab.xlabel('u')
# mlab.ylabel('v')
# mlab.zlabel('rho')
# mlab.plot3d(u, v, rho, tube_radius = 0.02, line_width = 2.0)

print('Mean u: ', np.mean(u))
print('Mean v: ', np.mean(v))

fig, ax = plt.subplots(1, 2)
ax[0].plot( time, u, label = 'u' )
ax[0].plot( time, v, label = 'v')
ax[0].plot( time, w, label = 'w')
ax[0].legend()
ax[0].set_xlabel('time')

ax[1].plot( time, x)
ax[1].set_ylabel('x')
ax[1].set_xlabel('time')
plt.show()
# mlab.show()