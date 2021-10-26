import numpy as np
import matplotlib.pyplot as plt
from mayavi import mlab
from tvtk.api import tvtk


delta = 0.025
a = 2.0
b = 30.0
sigma = 10.0
epsilon = 0.005
f1 = lambda rho, u, v: epsilon*(-a*u + b*(1 - u)*np.exp(-sigma*rho**2))
f2 = lambda rho, u, v: epsilon*(-a*v + b*(1 - v)*np.exp(-sigma*(rho-1)**2))
g = lambda rho, u, v: -4.0*(rho*(rho - 1)*(2*rho - 1) + (1-u)*rho/2.0 + (1-v)*(rho - 1)/2.0)
u = lambda rho, v: 2.0*(rho*(rho - 1)*(2*rho - 1) + (1 - v)*(rho - 1)/2.0)/rho + 1.0
ux, vx, rhox = np.mgrid[0:1:50j, 0:1:50j, 0.0:1.0:50j]
values = g(rhox, ux, vx)

mlab.figure(size = (800, 600), bgcolor = (1,1,1), fgcolor = (0, 0, 0))
mlab.clf()
mlab.contour3d(ux, vx, rhox, values, contours=[0], colormap = 'Blues', opacity = 0.8, line_width = 1.0)
# mlab.flow( f1(rhox, ux, vx), f2(rhox, ux, vx), g(rhox, ux, vx), 
#            extent = [0, 1, 0, 1, 0, 1], reset_zoom = True, seed_visible = False,
#            line_width = 1.0, seed_resolution = 100)
mlab.axes(extent = [0, 1, 0, 1, 0, 1], xlabel = 'u', ylabel = 'v', zlabel = 'rho')
mlab.outline(color = (0, 0, 0))
mlab.show()