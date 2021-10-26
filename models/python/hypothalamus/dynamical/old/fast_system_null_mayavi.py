from mayavi import mlab
import matplotlib.pyplot as plt  

mlab.clf()
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm

a = 4.0
b = 10.0
sigma = 10.0
delta = 0.025
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
A1 = lambda rho, u, v: -a*u + b*(1 - u)*np.exp(-sigma*rho**2)
A2 = lambda rho, u, v: -a*v + b*(1 - v)*np.exp(-sigma*(rho-1)**2)
F = lambda rho, u, v: -4*(rho*(rho - 1)*(2*rho - 1) + (1-u)*rho/2.0 + (1-v)*(rho - 1)/2.0)
gz = lambda t: t
gy = lambda t: b*np.exp(-sigma*(t-1)**2)/(a + b*np.exp(-sigma*(t-1)**2))
gx = lambda t: b*np.exp(-sigma*t**2)/(a + b*np.exp(-sigma*t**2))
rho = np.arange(0.0, 1.0, delta)
ux, vx, rhox = np.mgrid[0:1:50j, 0:1:50j, 0.0:1.0:50j]
values = F(rhox, ux, vx)
mlab.contour3d(ux, vx, rhox, values, contours=[0], opacity = 0.5)
values = A1(rhox, ux, vx)
mlab.contour3d(ux, vx, rhox, values, contours=[0], opacity = 0.5)

values = A2(rhox, ux, vx)
mlab.contour3d(ux, vx, rhox, values, contours=[0], opacity = 0.5)
mlab.axes()
mlab.xlabel('u')
mlab.ylabel('v')
mlab.zlabel('rho')

# mlab.surf(X1, A2, R)
# surf1 = ax.plot_wireframe(R, X2, A1, color = 'red')
# surf2 = ax.plot_wireframe(R, A2, X1, color = 'blue')
mlab.plot3d(gx(rho), gy(rho), gz(rho), color = (0,0,0), line_width = 0.5 )
mlab.show()
# ax.set_zlim(-1, 1)
# ax.set_xlabel('rho')
# ax.set_ylabel('a2')
# ax.set_zlabel('a1')

# fig, ax2 = plt.subplots(subplot_kw={"projection": "3d"})

# bs = range(10, 30)
# for i in range(len(bs)):
#     gx = lambda t: t
#     gy = lambda t, b: b*np.exp(-sigma*(t-1)**2)/(a + b*np.exp(-sigma*(t-1)**2))
#     gz = lambda t, b: b*np.exp(-sigma*t**2)/(a + b*np.exp(-sigma*t**2))
#     c = (10*(len(bs) - i))/255.0
#     print c
#     ax2.plot3D(gx(rho), gy(rho, bs[i]), gz(rho, bs[i]), color = (c, c,c), linewidth = 2.0 )
# ax2.set_xlabel('rho')
# ax2.set_ylabel('a2')
# ax2.set_zlabel('a1')    
# ax2.view_init(0, 0)
# plt.show()