from mayavi import mlab
import matplotlib.pyplot as plt  

mlab.clf()
import numpy as np 

T = 550000
h = 0.001
a = 0.5
b = 0.5
epsilon = 0.05
c = 0.4
eta = lambda rho: np.exp(-(rho)**2/(2*c**2))
nrho = lambda rho, v: -2.0*(rho**3 + (rho-1.0)*v/2.0 - rho)/(rho + 1.0)
nu = lambda rho: (b - eta(rho+1))/a
u = np.zeros(T)
v = np.zeros(T)
rho = np.zeros(T)
time = np.zeros(T)
# Maps
f = lambda rho, u, v: -rho**3 - (rho + 1.0)*u/2.0 - (rho - 1.0)*v/2.0 + rho
g1 = lambda rho, u, v: epsilon*(a*(1 - u) - eta(rho+1))
g2 = lambda rho, u, v: epsilon*(b*(1 - v) - eta(rho-1))
# Initial conditions
u[0] = 0.8
v[0] = 0.8
rho[0] = -0.1

for i in range(T-1):
    rho[i+1] = rho[i] + h*f(rho[i], u[i], v[i])
    u[i+1] = u[i] + h*g1(rho[i], u[i], v[i])
    v[i+1] = v[i] + h*g2(rho[i], u[i], v[i])
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

plt.plot( time, u, label = 'u' )
plt.plot( time, v, label = 'v')
plt.legend()
plt.show( block = False )
mlab.show()