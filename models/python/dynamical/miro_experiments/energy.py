import numpy as np 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
import matplotlib.animation as animation



fig = plt.figure()
ax = fig.gca(projection='3d')

# Make data.
a = 1.
b = 0.5
c = 0.5
d = 0.
mu1 = 0.0
mu2 = 0.0
# X = np.arange(-1.0, 1.0, 0.1)
# Y = np.arange(0.0, 2.0, 0.1)
X = np.linspace(-1., 1., 100)
Y = np.linspace(0.0, 2.0, 100)
N = len(X)
nmax = 40
mus = np.linspace(0., 5.0, nmax)
zarray = np.zeros((N, N, nmax))
X, Y = np.meshgrid(X, Y)

def update_plot(frame_number, zarray, plot):
    plot[0].remove()
    plot[0] = ax.plot_surface(X, Y, zarray[:,:,frame_number], cmap="magma")

for i in range(len(mus)):
    
    mu1 = mus[i]
    mu2 = 0.0
    
    # zarray[:,:,i] = -a*np.multiply(X, X) - (b + c)*np.multiply(X, Y) - d*np.multiply(Y, Y) - mu1**2*np.log(np.abs(np.cos(X))) - mu2**2*np.log(np.abs(np.cos(Y)))
    # zarray[:,:,i] = - (b + c)*np.multiply(X, Y) + mu1*Y
    zarray[:,:,i] = np.multiply(Y,np.arctan(np.multiply(Y,X)*np.pi/2.)) - np.tan(np.pi*X/2.)

# Plot the surface.
surf = [ax.plot_surface(X, Y, zarray[:,:,0], cmap=cm.coolwarm,
                    linewidth=0, antialiased=False)]

# Customize the z axis.
ax.set_zlim(-1.01, 2.01)

    # Add a color bar which maps values to colors.
    # fig.colorbar(surf, shrink=0.5, aspect=5)
animate = animation.FuncAnimation(fig, update_plot, nmax, fargs=(zarray, surf))

fig2, ax = plt.subplots(1,1)
# x = np.linspace(-1., 1., nx)
# y = np.linspace(-1., 1., ny)
# X, Y = np.meshgrid(x, y)
Z = np.multiply(Y,np.arctan(np.multiply(Y,X)*np.pi/2.)) - np.tan(np.pi*X/2.)
CS = ax.contour(X, Y, Z, levels = [ -0.3, -0.01, 0., 0.01, 0.3])
ax.set_xlabel('v1')
ax.set_ylabel('\lambda')
plt.show()