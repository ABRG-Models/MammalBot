import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm

a = 4.0
b = 10.0
sigma = 10.0

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
a1 = lambda a2, rho: b*np.exp(-sigma*rho**2)/(a + b*np.exp(-sigma*rho**2))
a2 = lambda a1, rho: b*np.exp(-sigma*(rho-1)**2)/(a + b*np.exp(-sigma*(rho-1)**2))
F = lambda rho, u, v: -4*(rho*(rho - 1)*(2*rho - 1) + u*rho/2.0 + v*(rho - 1)/2.0)
gx = lambda t: t
gy = lambda t: b*np.exp(-sigma*(t-1)**2)/(a + b*np.exp(-sigma*(t-1)**2))
gz = lambda t: b*np.exp(-sigma*t**2)/(a + b*np.exp(-sigma*t**2))
delta = 0.025
x2 = np.arange(0, 1, delta)
x1 = np.arange(0, 1, delta)
rho = np.arange(0.0, 0.8, delta)
X1,R = np.meshgrid(x1, rho)
X2,R = np.meshgrid(x2, rho)
A1 = a1(X2, R)
A2 = a2(X1, R)

# surf1 = ax.plot_surface(R, X2, A1, cmap = cm.bone)
# surf2 = ax.plot_surface(R, A2, X1, cmap = cm.bone)
surf1 = ax.plot_wireframe(R, X2, A1, color = 'red')
surf2 = ax.plot_wireframe(R, A2, X1, color = 'blue')
ax.plot3D(gx(rho), gy(rho), gz(rho), 'k', linewidth = 2.0 )

ax.set_zlim(-1, 1)
ax.set_xlabel('rho')
ax.set_ylabel('a2')
ax.set_zlabel('a1')

fig, ax2 = plt.subplots(subplot_kw={"projection": "3d"})

bs = range(10, 30)
for i in range(len(bs)):
    gx = lambda t: t
    gy = lambda t, b: b*np.exp(-sigma*(t-1)**2)/(a + b*np.exp(-sigma*(t-1)**2))
    gz = lambda t, b: b*np.exp(-sigma*t**2)/(a + b*np.exp(-sigma*t**2))
    c = (10*(len(bs) - i))/255.0
    print c
    ax2.plot3D(gx(rho), gy(rho, bs[i]), gz(rho, bs[i]), color = (c, c,c), linewidth = 2.0 )
ax2.set_xlabel('rho')
ax2.set_ylabel('a2')
ax2.set_zlabel('a1')    
ax2.view_init(0, 0)
plt.show()