import numpy as np  
import matplotlib.pyplot as plt 

T = 50000
v = 0.05
w = 0.05
h = 0.01
a = 0.3
b = 0.3
epsilon = 0.05
c = 0.2
eta = lambda rho: np.exp(-(rho)**2/(2*c**2))
nrho = lambda rho, v: (-(rho**3 - rho)*(3.0*rho**2 - 1) - v*rho - w*(rho - 1))/(rho + 1.0)
nu = lambda rho: (b - eta(rho+1))/a
u = np.zeros(T)
rho = np.zeros(T)
time = np.zeros(T)
# Maps
f = lambda rho, u, v: -(rho**3 - rho)*(3.0*rho**2 - 1) - u*(rho + 1) - v*rho - w*(rho - 1)
g1 = lambda rho, u, v: epsilon*(b - a*u - eta(rho+1))
# Initial conditions
u[0] = 0.0
rho[0] = -0.4

for i in range(T-1):
    rho[i+1] = rho[i] + h*f(rho[i], u[i], v)
    u[i+1] = u[i] + h*g1(rho[i], u[i], v)
    time[i+1] = time[i] + h

fig, ax = plt.subplots(1, 2)
rhos = np.linspace(-0.99, 1.5, 100)
ax[0].plot( rhos, nrho(rhos, v), color = [0.5, 0.5, 0.5], linewidth = 2.0)
ax[0].plot( rhos, nu(rhos), color = [0.5, 0.5, 0.5], linewidth = 2.0)
ax[0].plot( rho, u, 'k' )
ax[0].set_ylabel('u')
ax[0].set_xlabel('rho')
ax[0].axis([-2, 2, -1.0, 1.5])

ax[1].plot( time, u )
plt.show()