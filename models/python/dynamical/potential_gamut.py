import numpy as np
import matplotlib.pyplot as plt  

def potential1(ax):
    D = 1.0
    E = 2.0
    a = 1.0
    b = 1.0
    x0 = 3.0

    xb = 0.0

    V_morse_p = lambda x: D*(1 - np.exp(b*(x - x0)))**2 - D
    V_morse_m = lambda x: D*(1 - np.exp(-b*(x + x0)))**2 - D
    V_gauss = lambda x: E*np.exp(-a*(x - xb)**2)
    V = lambda x: V_morse_m(x) + V_gauss(x) + V_morse_p(x)

    x = np.linspace(-4.0, 4.0, 100)
    ax.plot(x, V(x))


def potential2():
    a = 0.1
    b = 0.1
    V = lambda x: (1 - x)**2*(1 + x)**2 + a*(1 - x)**2 + b*(1 - x)**2
    x = np.linspace(-2, 2, 100)
    plt.plot(x, V(x))

def potential3():
    # Not double?
    a = 3.9
    c = 0.1
    p = lambda x: x**2
    # V = lambda x: ((1 + a)/2.0)*p(c/2.0 + x) + ((1 - a)/2.0)*p(c/2.0 - x)
    V = lambda x: (1 - np.cos(x))*np.cos(c/2.0)
    x = np.linspace(-2, 2, 100)
    plt.plot(x, V(x))

def potential4():
    a = 0.5
    b = 0.5
    V = lambda x: (1.0/4.0)*x**4 - a*x - (1.0/2.0)*b*x**2
    x = np.linspace(-2, 2, 100)
    plt.plot(x, V(x))

def potential5():
    h = 0.5
    c = 1.0
    V = lambda x: -(1.0/4.0)*h**4*x**2 + (1.0/2.0)*c**2*x**4  
    x = np.linspace(-2, 2, 100)
    plt.plot(x, V(x))

def potential6():
    a = 1.0
    b = 1.0
    Vmax = 1.0
    V = lambda x: (Vmax/(b**4))*((x - a/2.0)**2 - b**2)**2
    x = np.linspace(-2, 2, 100)
    plt.plot(x, V(x))

f1 = lambda x: (1.0/np.cos(x))-1.0
f2 = lambda x: (np.exp(x) + np.exp(-x))/2.0 - 1
f3 = lambda x: np.abs(x)**1.9
f4 = lambda x: x**2
f5 = lambda x: np.sqrt(1 + x**2) - 1.0
f6 = lambda x: 1.0 - np.cos(x)
f7 = lambda x: 1.0 - np.exp(-x**2)

x = np.linspace(-1.5, 1.5, 100)
# Three wells
a = 0.3
b = 0.3
c = 0.0
V = lambda x: (1 + x)**2*x**2*(1 - x)**2 + a*(1 + x)**2 + b*x**2 + c*(1 - x)**2
plt.plot(x, V(x))
plt.show()


