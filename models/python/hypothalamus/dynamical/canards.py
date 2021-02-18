import numpy as np 
import matplotlib.pyplot as plt 

fig, ax = plt.subplots(3,1)

# plot critical manifold
k = 0.2
fc = lambda x: (-2.0*x**3 - (k-2)*x + k)/(x + 1)
gc = lambda x: x**3/3.0 - x
rho = np.linspace(-0.9, 1, 100)

ax[0].plot(rho, fc(rho))
ax[0].set_title('Critical surface')
ax[0].set_xlabel('rho')
ax[0].set_xlabel('u')

# plot fase space

epsilon = 0.1
alpha = 1.0
beta = 0.1
c = 1.0
x1 = 1.0
x2 = -1.0
rho1 = -1.0
rho2 = 1.0

sigma_c = 1.0
eta = lambda rho: np.exp(-(rho)**2/(2*sigma_c**2))/np.sqrt(2*np.pi*sigma_c**2)

xi = lambda x: (x - x1) == 0

f = lambda u, rho, x: c - alpha*u + rho + 0.0*xi(x)
g = lambda u, rho, x: (-rho**3 - (u + k -2)*rho/2.0 - (u - k)/2.0)/epsilon
h = lambda u, rho, x: (-beta*(x - x1)*eta(rho - rho1) - beta*(x - x2)*eta(rho - rho2))/epsilon

T = 5000
u = np.zeros(T)
rho = np.zeros(T)
x = np.zeros(T)
t = np.zeros(T)

u[0] = 0.1
rho[0] = 0.0
x[0] = -1.0
dt = 0.01

for i in range(T-1):
    u[i+1] = u[i] + dt*f(u[i], rho[i], x[i])
    rho[i+1] = rho[i] + dt*g(u[i], rho[i], x[i])
    x[i+1] = x[i] + dt*h(u[i], rho[i], x[i])
    t[i+1] = t[i] + dt


ax[1].plot(t, u)
ax[1].set_title('time series')
ax[1].set_ylabel('u')

ax[2].plot(t, x)
ax[2].set_title('time series')
ax[2].set_ylabel('x')
plt.show()