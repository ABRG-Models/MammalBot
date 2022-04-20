import numpy as np  
import matplotlib.pyplot as plt 

T = 250000
alpha = 0.5
beta = 0.5
epsilon = 0.01
h = 0.005
# v = 0.0
a = np.zeros(T)
b = np.zeros(T)
u = np.zeros(T)
v = np.zeros(T)
rho = np.zeros(T)
x = np.zeros(T)
time = np.zeros(T)
c1 = 0.2
c2 = 0.05
eta = lambda rho, c: np.exp(-(rho)**2/(2*c**2))
d1 = 1.0
d2 = 1.0
l = 0.5
x1 = -1.0
x2 = 1.0
G = 0.1
ga = lambda a, x: epsilon*(G - d1*eta(x - x1, c2))
gb = lambda b, x: epsilon*(G - d2*eta(x - x2, c2))
C0 = lambda rho, v: -2.0*(rho**3/3.0 + (rho-1.0)*v/2.0 - rho)/(rho + 1.0)
f = lambda rho, u, v, x: -rho**3/3.0 - (rho + 1.0)*u/2.0 - (rho - 1.0)*v/2.0 + rho
g1 = lambda rho, u, v, x, a: epsilon*(-alpha*(u - rho**2) - d1*eta(x - x1, c2))
g2 = lambda rho, u, v, x, b: epsilon*(-beta*(v - rho**2) - d2*eta(x - x2, c2))

fm = lambda rho, u, v, x: -u*eta(rho + 1, c1)*(x - x1)/10.0 - v*eta(rho - 1, c1)*(x - x2)/10.0
a[0] = 0.0
b[0] = 0.0
u[0] = 1.0
v[0] = 1.0
rho[0] = 0.5
x[0] = 0.0

for i in range(T-1):
    a[i+1] = a[i] + h*ga(a[i], x[i])
    b[i+1] = b[i] + h*gb(b[i], x[i])
    rho[i+1] = rho[i] + h*f(rho[i], u[i], v[i], x[i])
    u[i+1] = u[i] + h*g1(rho[i], u[i], v[i], x[i], a[i])
    v[i+1] = v[i] + h*g2(rho[i], u[i], v[i], x[i], b[i])
    x[i+1] = x[i] + h*fm(rho[i], u[i], v[i], x[i])
    time[i+1] = time[i] + h


fig, ax = plt.subplots(1, 3)
rhos = np.linspace(-0.9, 0.9, 100)
ax[0].plot( time, a )
ax[0].plot( time, b, 'k--')

ax[1].plot(time, u, label = 'u')
ax[1].plot(time, v, label = 'v')
ax[1].legend()

ax[2].plot(time, x)
ax[2].set_ylabel('x')
plt.show()