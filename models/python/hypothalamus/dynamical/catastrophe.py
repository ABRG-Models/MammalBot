import numpy as np
import matplotlib.pyplot as plt
import numpy.polynomial.polynomial as poly 
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

def add_embed_potential( x, y, a, b, low = 0.06, high = 0.2 ):
    U = lambda rho, a, b: rho**2*(1 - rho)**2 + a*rho**2 + b*(1 - rho)**2

    fig = plt.gcf()
    sa1 = fig.add_axes( [x, y, .08, .08] )
    sa1.set_xticks([])
    sa1.set_yticks([])

    rho = np.linspace( -0.1, 1.1, 100 )
    sa1.plot( rho, U(rho, a, b), linewidth = 2.0)
    sa1.plot( [0.5, 0.5], [low, high], 'k--')
    sa1.patch.set_alpha(0.7)
    sa1.set_ylim( low , high )

# Catastrophe surface
fig = plt.figure()
ax = fig.gca(projection='3d')

# Base space
A = np.arange(0, 2, 0.01)
B = np.arange(0, 2, 0.01)
A, B = np.meshgrid(A, B)
Z = -4*A**3 - 27*B**2
# Z[Z<0] = -1
# Z[Z==0] = 0
# Plot the surface.
surf = ax.plot_surface(A, B, Z, cmap = cm.viridis, rstride=1, cstride=1,
                       linewidth=0, antialiased=True, vmin=-1, vmax = 1)

plt.show()