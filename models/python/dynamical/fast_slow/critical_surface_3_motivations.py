from mayavi import mlab
import matplotlib.pyplot as plt  

mlab.clf()
import numpy as np 
rho1 = -1.0
rho2 = 1.0
rho3 = 3.0

mu = 2.0
uf = lambda u0, t: u0*np.exp(-mu*t)
vf = lambda v0, t: v0*np.exp(-mu*t)

w = 1.0
u, v, rho = np.mgrid[0:2:50j, 0:2:50j, -1.5:3.5:50j]
values = -2.0*((rho - rho1)*(rho - rho2)*(rho - rho3)*(3.0*rho**2 - 2.0*(rho1 + rho2 + rho3)*rho + rho1*rho2 + rho1*rho3 + rho2*rho3) + 
                       4.0*(1 - u)*(rho - rho1) + 20.0*(1 - v)*(rho - rho2) + 4.0*(1 - w)*(rho - rho3))
mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.5)
mlab.axes()
t = np.linspace(0, 100, 1000)
mlab.plot3d(uf(1.5, t), vf(1.5, t), 3.0*np.ones_like(t), tube_radius = 0.02, line_width = 2.0)


mlab.xlabel('u')
mlab.ylabel('v')
mlab.zlabel('rho')

mlab.show()