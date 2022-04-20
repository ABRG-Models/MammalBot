import numpy as np  
import matplotlib.pyplot as plt 

W = np.array([[0., 1.], [1., 0.]])
# g = lambda u, l:2.0*np.arctan(np.pi*l*u/2.0)/np.pi
g = lambda u, l:2.0*np.arctan(np.pi*np.multiply(l,u)/2.0)/(np.pi)
T =600
h = 0.01
NT = int(T/h)

u = np.zeros((2, NT))
t = np.zeros(NT)
tau = np.array([1., 1.])
l = [1.1, 1.1]

u[:,0] = [0.3, 0.0]
f = lambda u: -u + np.dot(W, g(u, l))

def F( x, y ):
    fx = -x + W[0, 0]*g(x, l[0]) + W[0, 1]*g(y, l[1])
    fy = -y + W[1, 0]*g(x, l[0]) + W[1, 1]*g(y, l[1])
    return fx, fy

nx, ny = 64, 64
x = np.linspace(-1., 1., nx)
y = np.linspace(-1., 1., ny)
X, Y = np.meshgrid(x, y)
U,V = F(X,Y)

for i in range(NT-1):
    u[:,i+1] = u[:,i] + h*f(u[:,i])
    t[i+1] = t[i] + h

fig, ax = plt.subplots(1, 2)
ax[0].plot(t, u[0,:], label = "v1")
ax[0].plot(t, u[1,:], label = "v2")
ax[0].legend()
ax[0].set_xlabel("time")

ax[1].quiver(X, Y, U, V)

## Countour plot
a = 0.5
b = 0.5
c = 0.5
d = 0.
mu1 = 1.0/l[0]
mu2 = 1.0/l[1]
# X = np.arange(-1.0, 1.0, 0.1)
# Y = np.arange(-1.0, 1.0, 0.1)
# X, Y = np.meshgrid(X, Y)
Z = -a*np.multiply(X, X) - (b + c)*np.multiply(X, Y) - d*np.multiply(Y, Y) - mu1*np.log(np.abs(np.cos(X))) - mu2*np.log(np.abs(np.cos(Y)))

CS = ax[1].contour(X, Y, Z, levels = [-0.15, -0.1, -0.01, 0., 0.05, 0.1, 0.2, 0.5, 1.0])
ax[1].set_xlabel("v1")
ax[1].set_ylabel("v2")



plt.show()
