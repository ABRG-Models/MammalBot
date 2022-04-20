from mayavi import mlab
import matplotlib.pyplot as plt  

mlab.clf()
import numpy as np 
rho1 = -1.0
rho2 = 1.0
rho3 = 3.0

ws = np.linspace(0, 2.0, 100)



w = ws[0]
u, v, rho = np.mgrid[0:2:50j, 0:2:50j, -1.5:3.5:50j]
values = -2.0*((rho - rho1)*(rho - rho2)*(rho - rho3)*(3.0*rho**2 - 2.0*(rho1 + rho2 + rho3)*rho + rho1*rho2 + rho1*rho3 + rho2*rho3) + 
                    4.0*(1 - u)*(rho - rho1) + 10.0*(1 - v)*(rho - rho2) + 4.0*(1 - w)*(rho - rho3))
mc = mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.5)

@mlab.animate
def anim():
    for i in range(len(ws)):
        w = ws[i]
        values = -2.0*((rho - rho1)*(rho - rho2)*(rho - rho3)*(3.0*rho**2 - 2.0*(rho1 + rho2 + rho3)*rho + rho1*rho2 + rho1*rho3 + rho2*rho3) + 
                    4.0*(1 - u)*(rho - rho1) + 10.0*(1 - v)*(rho - rho2) + 4.0*(1 - w)*(rho - rho3))
        mc.mlab_source.scalars = values
        yield

anim()
mlab.axes()
mlab.xlabel('u')
mlab.ylabel('v')
mlab.zlabel('rho')
mlab.show()