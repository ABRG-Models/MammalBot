from mayavi import mlab
import matplotlib.pyplot as plt  
mlab.clf()
import numpy as np 

def draw_critical2m( fold = True ):
    rho1 = -1.0
    rho2 = 1.0

    u, v, rho = np.mgrid[0:1:50j, 0:1:50j, -1.5:1.5:50j]
    # Critical manifold
    values = -2.0*((rho - rho1)*(rho - rho2)*(rho - (rho1+rho2)/2.0) + 
                        (1-u)*(rho - rho1)/2.0 + (1-v)*(rho - rho2)/2.0)
    mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.5)
    mlab.axes() 
    # Fold
    if fold:
        c = (rho1 + rho2)/2.0
        values = -2.0*(3.0*rho**2 - (u + v)/2.0)
        mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.5)
    
      
    mlab.xlabel('u')
    mlab.ylabel('v')
    mlab.zlabel('rho')