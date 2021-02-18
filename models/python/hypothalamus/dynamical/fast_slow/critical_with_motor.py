import numpy as np 
import matplotlib.pyplot as plt 

fig, ax = plt.subplots(1,2)
plt.ion()
plt.show(block = False)

v = 0.1
C0 = lambda x: 2.0*(-x**3 -(x-1)*v/2.0 + x)/(x + 1)

xx = np.linspace(-0.97,1.5,100)
h = 0.01
T = 200000
e = 0.1
alpha = 0.1
sigma_c = 0.09
eta = lambda rho: np.exp(-(rho)**2/(2*sigma_c**2))/np.sqrt(2*np.pi*sigma_c**2) 
f = lambda u, rho, x: e*(alpha*(1-u) - eta(rho + 1.0))
g = lambda u, rho, x: -rho**3 - (rho + 1)*u/2.0 - (rho - 1)*v/2.0 + rho
h = lambda u, rho, x: 


ax[0].plot(xx, C0(xx), linewidth = 2.0, color = (0.5, 0.5, 0.5))
ax[0].set_xlabel('rho')
ax[0].set_ylabel('u')

u = np.zeros(T)
rho = np.zeros(T)
t = np.zeros(T)
u[0] = 0.5
rho[0] = 0.5

for i in range(1,T):
    u[i] = u[i-1] + h*f(u[i-1], rho[i-1])
    rho[i] = rho[i-1] + h*g(u[i-1], rho[i-1])
    t[i] = t[i-1] + h
    # ax[0].plot(rho[i-1:i+1], u[i-1:i+1], 'k')
    # plt.pause(0.01)
ax[0].plot(rho, u, 'k')
ax[1].plot(t, u)
ax[1].set_xlabel('t')
ax[1].set_ylabel('u')
fig.suptitle('Slow - fast dynamics v = ' + str(v))
plt.ioff()
plt.show()

