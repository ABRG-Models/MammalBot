import numpy as np
import matplotlib.pyplot as plt 

rrate = 0.001
a_d = 1.0
b_d = 1.0
# eta1 = lambda rho: np.heaviside(-rho, 0.5)
# eta2 = lambda rho: np.heaviside(rho, 0.5)
g = lambda x: 1./(1. + np.exp(-10*x))
c = 0.45
g1 = lambda x: np.exp(-(x + 1)**2/(2*c**2))
g2 = lambda x: np.exp(-(x - 1)**2/(2*c**2))
eta1 = lambda rho: g1(rho)
eta2 = lambda rho: g2(rho)
alpha = 0.000
fa = lambda a, rho: -alpha*a +rrate*eta1(rho)
fb = lambda b, rho: -alpha*b + rrate*eta2(rho)

drive = lambda x: np.heaviside(x, 0.5)*x

def motivation_map( rho, a, b ):
    L = 0.1
    dU = lambda rho: rho**3 + (a + b - 2.0)*rho/2.0 + (a - b)/2.0
    #noise = np.random.normal(loc = 0.0, scale=self.diff_heat)/np.sqrt(h)
    dRho = -L*dU( rho ) #+ noise

    return dRho


def evolve( T, rho0, a0, b0 ):
    h = 0.01
    N = int(T/h)
    rho = np.zeros(N)    
    t = np.zeros(N)    
    a = np.zeros(N)
    b = np.zeros(N)
    u = np.zeros(N)
    v = np.zeros(N)
    rho[0] = rho0
    t[0] = 0
    a[0] = a0
    b[0] = b0
    u[0] = eta1(rho0)
    v[0] = eta2(rho0)

    for i in range(N-1):
        a[i+1] = a[i] + h*fa( a[i], rho[i] )
        b[i+1] = b[i] + h*fb( b[i], rho[i] )
        # print("a: {}, b: {}".format(drive(a_d - a[i]), drive(b_d - b[i])))
        rho[i+1] = rho[i] + h*motivation_map( rho[i], drive(a_d - a[i]), drive(b_d - b[i]) )
        u[i+1] = eta1( rho[i] )
        v[i+1] = eta2( rho[i] )
        t[i+1] = t[i] + h

    return a, b, rho, u, v, t

a0 = 0.01
b0 = 0.1
r = 0.01
rho0 = 0.2

a, b, rho, u, v, t = evolve( 3000, rho0, a0, b0 )


fig, ax = plt.subplots(1, 3)
# U1 = lambda rho, a, b: (rho - 1)**2*(rho + 1)**2 + a*(rho + 1)**2 + b*(rho - 1)**2
# U2 = lambda rho, a, b: rho**4 + 1 + (a-1)*rho**2 + (a-b)*rho + a + (b-1)*rho**2 + (a - b)*rho + b
# P1 = lambda rho, a, b: (a + b -2)*rho**2 + 2*(a-b)*rho + a + b + 1 #
# P2 = lambda rho, a, b: a*(rho + (a-b-2)/(2*a))**2 + a - (a-b)**2/(4*a) + 1.5
rhos = np.linspace(-1.1,1.1,100)
# F = lambda rho: rho**3 + m*rho + k
# Fl = lambda rho: m*rho + k
# plt.plot( rhos, F(rhos))
# plt.plot( rhos, Fl(rhos))
# plt.plot( rhos, U1(rhos, a, b) )
# plt.plot( rhos, P1(rhos, a, b) )
# plt.plot( rhos, P2(rhos, a, b) )
ax[0].plot( a, b, '.-' )
ax[0].set_xlabel('a')
ax[0].set_ylabel('b')
plt.grid()
ax[1].plot(t, rho)
ax[1].set_xlabel('time')
ax[1].set_ylabel('rho')
ax[2].plot(t, u)
ax[2].plot(t, v)
ax[2].set_xlabel('time')
ax[2].set_ylabel('tendency')

plt.figure()
plt.plot(rhos, g1(rhos))
plt.plot(rhos, g2(rhos))
plt.show()
