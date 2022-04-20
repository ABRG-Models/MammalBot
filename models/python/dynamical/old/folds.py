import numpy as np
import matplotlib.pyplot as plt
from mayavi import mlab
import motivational_system as ms
from scipy.optimize import fsolve
delta = 0.025
a = 4.0
b = 10.0
sigma = 10.0
t, X = ms.int_translated_system( a = 2.0, b = 100, T = 200000, x0 = [0.8, 0.9, 0.5])

# t, X = ms.int_system( a = 4.0, b = 10.0, T = 400000, x0 = [0.9, 0.2, 0.5])


ux, vx, rhox = np.mgrid[0:1:50j, 0:1:50j, -0.5:0.5:50j]
# ux, vx, rhox = np.mgrid[0:1:50j, 0:1:50j, -1.0:1.0:50j]
# values = F(rhox + 0.5, ux, vx)
# mlab.contour3d(ux, vx, rhox, values, contours=[0], opacity = 0.3)
mlab.figure(size = (800, 600), bgcolor = (1,1,1), fgcolor = (0, 0, 0))
mlab.clf()

values = ms.G_t(rhox, ux, vx)
mlab.contour3d(ux, vx, rhox, values, contours=[0], opacity = 0.5)
values = ms.fold_t(rhox, ux, vx)
mlab.contour3d(ux, vx, rhox, values, contours=[0], opacity = 0.5)

mlab.plot3d(X[0,:], X[1,:], X[2,:], color = (0,0,0), line_width = 2.0, tube_radius = None )
mlab.points3d(X[0,0], X[1,0], X[2,0], scale_factor=0.03, line_width = 0.1)
mlab.axes(extent = [0, 1, 0, 1, -0.5, 0.5], xlabel = 'u', ylabel = 'v', zlabel = 'rho')
# mlab.axes(extent = [0, 1, 0, 1, -1.0, 1.0], xlabel = 'u', ylabel = 'v', zlabel = 'rho')
# mlab.outline(color = (0, 0, 0))
mlab.show()