from mayavi import mlab
import matplotlib.pyplot as plt  
mlab.clf()
import numpy as np 

rho1 = -1.0
rho2 = 1.0

u, v, rho = np.mgrid[-2:2:50j, -2:2:50j, -1.5:1.5:50j]
# Critical manifold
# values = -4.0*((rho - rho1)*(rho - rho2)*(rho - (rho1+rho2)/2.0) + 
#                     (1-u)*(rho - rho1)/2.0 + (1-v)*(rho - rho2)/2.0)
# Raw
values = -4.0*((rho - rho1)*(rho - rho2)*(rho - (rho1+rho2)/2.0) + u/2.0+ v/2.0)

mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.5)
mlab.axes() 
mlab.xlabel('u')
mlab.ylabel('v')
mlab.zlabel('rho')


# Half
mlab.figure()
values = -4.0*((rho - rho1)*((rho - rho2)*(rho - (rho1+rho2)/2.0) + (1-u))/2.0)

mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.5)
mlab.axes() 
mlab.xlabel('u')
mlab.ylabel('v')
mlab.zlabel('rho')

# other Half
mlab.figure()
values = -4.0*((rho - rho2)*((rho - rho1)*(rho - (rho1+rho2)/2.0) + (1-v))/2.0)

mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.5)
mlab.axes() 
mlab.xlabel('u')
mlab.ylabel('v')
mlab.zlabel('rho')

# Two Halfs
mlab.figure()
values = -4.0*((rho - rho1)*((rho - rho2)*(rho - (rho1+rho2)/2.0) + (1-u))/2.0)

mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.5)
values = -4.0*((rho - rho2)*((rho - rho1)*(rho - (rho1+rho2)/2.0) + (1-v))/2.0)

mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.5)
mlab.axes() 
mlab.xlabel('u')
mlab.ylabel('v')
mlab.zlabel('rho')

mlab.show()