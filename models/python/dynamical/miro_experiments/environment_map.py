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
#fa = lambda a, rho: -alpha*a +rrate*eta1(rho)
#fb = lambda b, rho: -alpha*b + rrate*eta2(rho)
fa = lambda a, rho: 0.
fb = lambda b, rho: 0.
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
        rho[i+1] = rho[i] + h*motivation_map( rho[i], a[i], b[i] )
        u[i+1] = eta1( rho[i+1] )
        v[i+1] = eta2( rho[i+1] )
        t[i+1] = t[i] + h

    return a, b, rho, u, v, t

a0 = 0.6
b0 = 0.1
r = 0.01
rho0 = 0.2

xmin = -250
xmax = 250
sigma = 100.0
g = lambda x,y,x0,y0: np.exp(-(( x - x0)**2 + (y - y0)**2)/(2*sigma**2) )
Tmin = 15.0
Tmax = 45.0
T = lambda x,y : (((Tmax - Tmin)/(xmax - xmin))*(x - xmin) + Tmin)/Tmax

nn = 5
fig, ax = plt.subplots(nn,nn)
a0 = np.linspace(0, 1, nn)
b0 = np.linspace(0, 1, nn)

for i in range(nn):
    for j in range(nn):

        a, b, rho, u, v, t = evolve( 100, rho0, a0[i], b0[j] )
        print(rho[-1], ", u: ", u[-1], ", v: ", v[-1])
        xx = np.linspace(xmin, xmax, 100 )
        yy = np.linspace(xmin, xmax, 100 )
        X,Y = np.meshgrid(xx, yy)
        I1 = g(X,Y,0.,0.)
        I2 = T(X, Y)
        I = u[-1]*I1 + v[-1]*I2
        ax[nn-i-1,j].imshow(I, vmin = 0, vmax = 1)
    
plt.show()
