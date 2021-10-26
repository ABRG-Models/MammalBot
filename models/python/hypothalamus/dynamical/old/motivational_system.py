import numpy as np

epsilon = 0.001
h = 0.01

G = lambda rho, u, v: (rho*(rho - 1)*(2*rho - 1) + (1-u)*rho/2.0 + (1-v)*(rho - 1)/2.0)
fold = lambda rho, u, v: 3.0*rho**2 - 3.0*rho + 1 - (u + v)/4.0
G2 = lambda rho, u, v: rho**3 - (3.0/2.0)*rho**2 + (1 - (u + v)/4.0)*rho - (1 -v)/4.0
G_t = lambda rho, u, v: rho**3 + ((1 - u - v)/4.0)*rho + (v - u)/8.0
fold_t = lambda rho, u, v: 3.0*rho**2 + (1.0 - u - v)/4.0


def integrate( f, T, x0 ):
    X = np.zeros((len(x0), T))
    time = np.zeros(T)
    X[:,0] = x0

    for i in range(T-1):
        k1 = f(time[i], X[:,i])
        k2 = f(time[i] + h/2.0, X[:,i] + h*k1/2.0)
        k3 = f(time[i] + h/2.0, X[:,i] + h*k2/2.0)
        k4 = f(time[i] + h, X[:,i] + h*k3)
        X[:, i+1] = X[:,i] + h*(k1 + 2*k2 + 2*k3 + k4)/6.0
        time[i+1] = time[i] + h
        
    return time, X


def int_translated_system(a, b, T, sigma = 10.0, x0 = [0.0, 0.0, 0.0]):
    f1 = lambda u, v, rho: -a*u + b*(1 - u)*np.exp(-sigma*(rho + 0.5)**2)
    f2 = lambda u, v, rho: -a*v + b*(1 - v)*np.exp(-sigma*(rho - 0.5)**2)
    g = lambda u, v, rho: -4.0*(rho**3 + (1 - u - v)*rho/4.0 + (v - u)/8.0)

    f = lambda t, x: np.array([epsilon*f1(x[0], x[1], x[2]),
                               epsilon*f2(x[0], x[1], x[2]),
                               g(x[0], x[1], x[2])])

    t, X = integrate( f, T, x0 )
    return t, X

def int_system(a, b, T, sigma = 10.0, x0 = [0.0, 0.0, 0.0]):
    f1 = lambda u, v, rho: -a*u + b*(1 - u)*np.exp(-sigma*(rho)**2)
    f2 = lambda u, v, rho: -a*v + b*(1 - v)*np.exp(-sigma*(rho - 1.0)**2)
    g = lambda u, v, rho: -4.0*(rho*(rho - 1)*(2.0*rho - 1) + (1-u)*rho/2.0 + (1-v)*(rho - 1)/2.0)

    f = lambda t, x: np.array([epsilon*f1(x[0], x[1], x[2]),
                               epsilon*f2(x[0], x[1], x[2]),
                               g(x[0], x[1], x[2])])

    t, X = integrate( f, T, x0 )
    return t, X