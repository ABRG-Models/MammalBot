from mayavi import mlab
import matplotlib.pyplot as plt  

mlab.clf()
import numpy as np 

T = 750000
h = 0.001
a = 0.25
b = 0.25
epsilon = 0.05
c1 = 0.2
c2 = 0.05
d1 = 1.0
d2 = 1.0
eta = lambda rho, c: np.exp(-(rho)**2/(2*c**2))
w = 0.406
u = np.zeros(T)
v = np.zeros(T)
rho = np.zeros(T)
time = np.zeros(T)
# Maps
f = lambda rho, u, v, w: -(rho**3 - rho)*(3.0*rho**2 - 1) - u*(rho + 1) - v*rho - w*(rho - 1)
g1 = lambda rho, u, v: epsilon*(a*(1 - u) - eta(rho+1, c1))
g2 = lambda rho, u, v: epsilon*(b*(1 - v) - eta(rho, c1))
# Initial conditions
u[0] = 0.0
v[0] = 0.0
rho[0] = 0.9

for i in range(T-1):
    rho[i+1] = rho[i] + h*f(rho[i], u[i], v[i], w)
    u[i+1] = u[i] + h*g1(rho[i], u[i], v[i])
    v[i+1] = v[i] + h*g2(rho[i], u[i], v[i])
    time[i+1] = time[i] + h

ux, vx, rhox = np.mgrid[-2:2:50j, -2:2:50j, -1.5:1.5:50j]
values = -(rhox**3 - rhox)*(3.0*rhox**2 - 1) - ux*(rhox + 1) - vx*rhox - w*(rhox - 1)
mlab.contour3d(ux, vx, rhox, values, contours=[0], opacity = 0.5)
# values_u = a*(1.0 - ux) - eta(rhox+1, c1)
# mlab.contour3d(ux, vx, rhox, values_u, color = (0.7, 0.4, 0.4), contours=[0], opacity = 0.5)
# values_v = b*(1.0 - vx) - eta(rhox, c1)
# mlab.contour3d(ux, vx, rhox, values_v, color = (0.7, 0.4, 0.4), contours=[0], opacity = 0.5)
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

ax[1].plot( time, rho)
ax[1].set_ylabel('rho')
ax[1].set_xlabel('time')
plt.show( block = False )
mlab.show()