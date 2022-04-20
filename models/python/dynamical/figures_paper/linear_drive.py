from mayavi import mlab
import matplotlib.pyplot as plt  
mlab.clf()
import numpy as np

h = 0.005
T = 50000

def integrate( f, T, x0 ):
    X = np.zeros((len(x0), T))
    time = np.zeros(T)
    X[:,0] = x0

    for i in range(T-1):
        k1 = f(time[i], X[:,i])
        k2 = f(time[i] + h/2.0, X[:,i] + h*k1/2.0)
        k3 = f(time[i] + h/2.0, X[:,i] + h*k2/2.0)
        k4 = f(time[i] + h, X[:,i] + h*k3)
        X[:, i+1] = X[:,i] + h*(k1 + 2*k2 + 2*k3 + k4)/6.0
        time[i+1] = time[i] + h
        
    return time, X

rho1 = -1.0
rho2 = 1.0

u, v, rho = np.mgrid[0:1:50j, 0:1:50j, -1.5:1.5:50j]
# Critical manifold
values = -2.0*((rho - rho1)*(rho - rho2)*(rho - (rho1+rho2)/2.0) + 
                    u*(rho - rho1) + v*(rho - rho2))
mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.5)
mlab.axes()   
mlab.xlabel('u')
mlab.ylabel('v')
mlab.zlabel('rho')
# Plane U
u, v = np.mgrid[0:1:50j, 0:1:50j]

c = 0.3
a = -0.3

b = 0.95

d = -b
rhoPu = -a/b
rhoPv = -c/d

u0 = 0.1
v0 = 0.1

f = lambda t, x: np.array([0.1*(a + b*(x[2])), #(a*(1.0-x[0]) + b*x[2])
                           0.1*(c + d*(x[2])), #(c*(1.0-x[1]) - d*x[2])
                          -2.0*((x[2] - rho1)*(x[2] - rho2)*(x[2] - (rho1+rho2)/2.0) + 
                                x[0]*(x[2] - rho1) + x[1]*(x[2] - rho2))])

t, X = integrate( f, T, [u0, v0, 0.36] )

s = mlab.mesh(u, v, rhoPu*np.ones_like(u), opacity = 0.5, color = (0.7, 0.0, 0.0))
s = mlab.mesh(u, v, rhoPv*np.ones_like(u), opacity = 0.5)

u, rho = np.mgrid[0:1:50j, -1:1:50j]
k = u0 + v0
s = mlab.mesh(u, k - u, rho, opacity = 0.5, color = (0.2, 0.2, 0.2))

# plt.plot(X[0,:] - X[1,:])
# plt.show()

mlab.plot3d(X[0, :], X[1,:], X[2,:], tube_radius = 0.01, line_width = 1.0)

mlab.show()