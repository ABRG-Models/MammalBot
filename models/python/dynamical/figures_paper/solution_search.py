import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib import cm

matplotlib.rcParams['figure.figsize'] = [12, 7]
matplotlib.rcParams['text.usetex'] = False
h = 0.005

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

def solve( a, b, sigma ):
    
    mu1 = a
    mu2 = a
    mu3 = a
    rho1 = -1.0
    rho2 = -0.0
    rho3 = 1.0
    T = 100000

    b1 = b
    b2 = b
    b3 = b
    e = 0.01

    g = lambda u, rho, ri, mu, b: -mu*u + b*(1.0 - u)*np.exp(-sigma*(rho - ri)**2)

    f = lambda t, x: np.array([e*g(x[0], x[3], rho1, mu1, b1),
                    e*g(x[1], x[3], rho2, mu2, b2),
                    e*g(x[2], x[3], rho3, mu3, b3),
                    -2.0*((x[3] - rho1)*(x[3] - rho2)*(x[3] - rho3)*((x[3] - rho1)*(x[3] - rho2) + (x[3] - rho1)*(x[3] -rho3) +
                        (x[3] - rho2)*(x[3] - rho3)) + (1.0 - x[0])*(x[3] - rho1) + (1.0 - x[1])*(x[3] - rho2) + (1.0 - x[2])*(x[3] - rho3))])

    t, X = integrate( f, T, [0.0, 0.0, 1.0, 1.0] )

    return t, X

a_values = np.linspace(0.1, 10, 20)
b_values = np.linspace(5.0, 60, 20)
sigma_values = np.linspace(1.0, 30.0, 20)
successes = []

for i in range(len(a_values)):
    for j in range(len(b_values)):
        for k in range(len(sigma_values)):
            t, X = solve(a_values[i], b_values[j], sigma_values[k])

            osc = np.sum(X[3,X[3,:] > 0.2])/t[-1]
            print( "values (%d, %d, %d) : %.2f"%(i, j, k, osc))
            
            if(osc > 0.3):
                successes += [(a_values[i], b_values[i], sigma_values[i])]

print(successes)