import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

### Plots the fold curve for
# some given locations of the motivations
def plot_fold(ax):
    
    L1 = lambda u, v: np.sqrt((1.0/6.0)*(u + v))
    L2 = lambda u, v: np.sqrt((1.0/6.0)*(u + v))


    u = np.arange(0.0, 1, 0.01)
    v = np.arange(0.0, 1, 0.01)
    U, V = np.meshgrid(u, v)
    Z1 = L1(U, V)
    Z2 = L2(U, V)
    
    

    ax.plot_surface(U, V, Z1, rstride=1, cstride=1,
                       linewidth=0, antialiased=True, vmin=-1, vmax = 1)
    ax.plot_surface(U, V, Z2, rstride=1, cstride=1,
                       linewidth=0, antialiased=True, vmin=-1, vmax = 1)
    # ax.contour(U, V, Z, [0], linewidths = [2.0], colors = ['k'])

def plot_fold_projection( ax, color = 'k' ):
    U = np.arange(0, 1, 0.01)
    V = np.arange(0, 1, 0.01)
    U, V = np.meshgrid(U, V)
    X = U + V
    Y = U - V
    Z = 2.0*X**3 - 27.0*Y**2
    ax.contour(U, V, Z, [ 0], linewidths = [2.0, 2.0], colors = [color])

def plot_fold_projection_rotated( ax ):
    X = np.arange(0, 2, 0.01)
    Y = np.arange(-1, 1, 0.01)
    X, Y = np.meshgrid(X, Y)
    
    Z = 2.0*X**3 - 27.0*Y**2
    ax.contour(X, Y, Z, [ 0], linewidths = [2.0, 2.0], colors = ['k'])