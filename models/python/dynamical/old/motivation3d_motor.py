from mayavi import mlab
import matplotlib.pyplot as plt  

mlab.clf()
import numpy as np 

T = 850000
h = 0.001
a = 0.5
b = 0.5
x1 = -1.0
x2 = 1.0
x3 = 2.0
epsilon = 0.1
c1 = 0.2
c2 = 0.05
d1 = 1.0
d2 = 1.0
eta = lambda rho, c: np.exp(-(rho)**2/(2*c**2))
# xi = lambda x : 0.5 - 1.0/(1.0 + np.exp(-x))
xi = lambda x : x/np.max([0.001, np.abs(x)])
u = np.zeros(T)
v = np.zeros(T)
rho = np.zeros(T)
x = np.zeros(T)
time = np.zeros(T)
# Maps
f = lambda rho, u, v, x: (-rho**3 - (rho + 1.0)*u/2.0 - (rho - 1.0)*v/2.0 + rho) 
g1 = lambda rho, u, v, x: epsilon**2*(a*(1.0 - u) - d1*eta(x - x1, c2))
g2 = lambda rho, u, v, x: epsilon**2*(b*(1.0 - v) - d2*eta(x - x2, c2))
fm = lambda rho, u, v, x: epsilon*(-eta(rho + 1, c1)*xi(u)*xi(x - x1) - eta(rho - 1, c1)*xi(v)*xi(x - x2))
# Initial conditions
u[0] = 0.8
v[0] = 0.8
rho[0] = -0.1
x[0] = 0.0

for i in range(T-1):
    rho[i+1] = rho[i] + h*f(rho[i], u[i], v[i], x[i])
    u[i+1] = u[i] + h*g1(rho[i], u[i], v[i], x[i])
    v[i+1] = v[i] + h*g2(rho[i], u[i], v[i], x[i])
    x[i+1] = x[i] + h*fm(rho[i], u[i], v[i], x[i])
    time[i+1] = time[i] + h

ux, vx, rhox = np.mgrid[-1:2:50j, -1:2:50j, -1.5:1.5:50j]
values = -rhox**3 - (rhox + 1.0)*ux/2.0 - (rhox - 1.0)*vx/2.0 + rhox
mlab.contour3d(ux, vx, rhox, values, contours=[0], opacity = 0.5)
mlab.axes()
mlab.xlabel('u')
mlab.ylabel('v')
mlab.zlabel('rho')
mlab.plot3d(u, v, rho, tube_radius = 0.02, line_width = 2.0)

print('Mean u: ', np.mean(u))
print('Mean v: ', np.mean(v))

fig, ax = plt.subplots(1, 2)
ax[0].plot( time, u, label = 'u' )
ax[0].plot( time, v, label = 'v')
ax[0].legend()
ax[0].set_xlabel('time')

ax[1].plot( time, x)
ax[1].set_ylabel('x')
ax[1].set_xlabel('time')
plt.show( block = False )
mlab.show()