import numpy as np
import matplotlib.pyplot as plt 


def motivation_map( rho, a, b ):
    L = 1.0
    dU = lambda rho: rho**3 + (a + b - 2.0)*rho/2.0 + (a - b)/2.0
    #noise = np.random.normal(loc = 0.0, scale=self.diff_heat)/np.sqrt(h)
    dRho = -L*dU( rho ) #+ noise
    return dRho


def evolve( T, rho0, a0, b0 ):
    h = 0.01
    N = int(T/h)
    rho = np.zeros(N)    
    t = np.zeros(N)
    rho[0] = rho0
    t[0] = 0

    for i in range(N-1):
        rho[i+1] = rho[i] + h*motivation_map( rho[i], a0, b0 )
        t[i+1] = t[i] + h

    return rho, t

a0 = 0.4
b0 = 0.4
rho0 = -0.5
T = 100

rho, t = evolve( T, rho0, a0, b0 )


fig, ax = plt.subplots(1, 2)
rhos = np.linspace(-1.1,1.1,100)
U1 = lambda rho, a, b: (rho - 1)**2*(rho + 1)**2 + a*(rho + 1)**2 + b*(rho - 1)**2
# U2 = lambda rho, a, b: rho**4 + 1 + (a-1)*rho**2 + (a-b)*rho + a + (b-1)*rho**2 + (a - b)*rho + b
# P1 = lambda rho, a, b: (a + b -2)*rho**2 + 2*(a-b)*rho + a + b + 1 #
# P2 = lambda rho, a, b: a*(rho + (a-b-2)/(2*a))**2 + a - (a-b)**2/(4*a) + 1.5
# F = lambda rho: rho**3 + m*rho + k
# Fl = lambda rho: m*rho + k
# plt.plot( rhos, F(rhos))
# plt.plot( rhos, Fl(rhos))
# plt.plot( rhos, U1(rhos, a, b) )
# plt.plot( rhos, P1(rhos, a, b) )
m = (a0 + b0 - 2.)/2.
k = (a0 - b0)/2.
c = rho0
rho_1 = lambda t: - (rho0**2 + m)*rho0*t + (3*rho0**2 + m)*k*t**2/2.0 - 3*rho0*k**2*t**3/3.0 + k**3*t**4/4.0
r1 = (c**3 + m*c)
r2 = 3*c**2 + m
rho_2 = lambda t: (3*c**2*r1 + m*r1)*t**2/2.0 - (3*c**2*r2*k/2.0 + 6*c*k*r1 + m*r2*k/2.0)*t**3/3.0 + \
                  (9*c**3*k**2/3.0 + 6*c*k**2*r2/2.0 + k**2*r1 + 3*m*c*k**2/3.0)*t**4/4.0 - \
                  (3*c**2*k**3/4.0 + 3*k**3*r2/2.0 + 18*c**2*k**3/3.0 + m*k**3/4.0)*t**5/5.0 + \
                  (6*c*k**4/4.0 + 9*c*k**4/3.0)*t**6/6.0 - (3*k**5/4.0)*t**7/7.0
rho_t = lambda t: rho0 - k*t + rho_1(t) + rho_2(t)
ax[0].plot( t, rho )
ax[0].plot( t, rho_t(t))
ax[1].plot(rhos, U1(rhos, a0, b0))
ax[0].set(xlim=(0, T), ylim=(-2.5, 2.5))
plt.show()
