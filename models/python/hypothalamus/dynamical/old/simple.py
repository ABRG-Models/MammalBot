import numpy as np  
import matplotlib.pyplot as plt 

T = 30000
# v = 0.02906
# v = 0.617085
v = 0.99
h = 0.01
a = 0.5
b = 0.5
epsilon = 0.05
c = 0.4
eta = lambda rho: np.exp(-(rho)**2/(2*c**2))
nrho = lambda rho, v: -2.0*(rho**3 + (rho-1.0)*v/2.0 - rho)/(rho + 1.0)
nu = lambda rho: (b - eta(rho+1))/a
u = np.zeros(T)
rho = np.zeros(T)
time = np.zeros(T)
# Maps
f = lambda rho, u, v: -rho**3 - (rho + 1.0)*u/2.0 - (rho - 1.0)*v/2.0 + rho
g1 = lambda rho, u, v: epsilon*(b - a*u - eta(rho+1))
# Initial conditions
u[0] = 0.0
rho[0] = -0.0

for i in range(T-1):
    rho[i+1] = rho[i] + h*f(rho[i], u[i], v)
    u[i+1] = u[i] + h*g1(rho[i], u[i], v)
    time[i+1] = time[i] + h

fig, ax = plt.subplots(1, 2)

# X, Y = np.meshgrid(np.arange(-0.6, 0.6, 0.1), np.arange(-0.2, 1.0, .1))
# U = f(X, Y, v)/epsilon #rho
# V = g1(X, Y, v)/epsilon #u
# q = ax[0].quiver(X, Y, U, V, units='x', pivot='tip')#, width=0.022, scale=1 / 0.40)
rhos = np.linspace(-0.99, 1, 100)
ax[0].plot( rhos, nrho(rhos, v), color = [0.8, 0.5, 0.5], linewidth = 3.0)
ax[0].plot( rhos, nu(rhos), color = [0.5, 0.5, 0.8], linewidth = 3.0)
ax[0].plot( rho[0], u[0], 'k.', linewidth = 3.0)
ax[0].plot( rho, u, 'k' )
ax[0].plot( [-1, -1], [-1.5, 1.5], 'k--')
ax[0].set_ylabel('u')
ax[0].set_xlabel(r'$\rho$')
ax[0].text(0.5, nu(0.5)+0.05, r'$u_0$')
ax[0].text(0.95, nrho(0.9, v), r'$\rho_0$')
ax[0].axis([-2, 2, -1.0, 1.5])

ax[1].plot( time, u, label = 'u')
ax[1].plot( time, rho, label = r'$\rho$' )
ax[1].legend()
ax[1].set_xlabel('time')
plt.show()