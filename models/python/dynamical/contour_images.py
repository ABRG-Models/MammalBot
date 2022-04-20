import numpy as np
import matplotlib.pyplot as plt 
from scipy.optimize import root
from mpl_toolkits.mplot3d import Axes3D 

U = lambda rho, a, b: (1.0/4.0)*rho**2*(1 - rho)**2 + a*rho**2 + b*(1 - rho)**2

def motivation_map( rho, a, b ):
    L = 10.0
    dU = lambda rho: (1.0/2.0)*(rho*((1-rho)**2 + a) - (1-rho)*(rho**2 + b))

    #noise = np.random.normal(loc = 0.0, scale=self.diff_heat)/np.sqrt(h)
    dRho = -L*dU( rho ) # noise

    return dRho

def evolve( T, rho0, a, b ):
    h = 0.01
    N = int(T/h)
    rho = rho0

    for i in range(N):
        rho += h*motivation_map( rho, a, b )

    return rho

# First graph : color bar as the line evolves
a = 0.0
Nb = 100
Nrho = 100
bs = np.linspace(0, 1, Nb)
rhos = np.linspace(-0.5, 1.5, Nrho)

X = np.zeros((Nrho, Nb))

for i in range(Nb):
    b = bs[i]
    for j in range(Nrho):
        X[j,i] = U(rhos[j], a, b)

plt.figure()
plt.imshow( X, interpolation='nearest', origin='lower', vmin = 0.0, vmax = 0.1)
plt.colorbar()
lbl_range_x = np.arange(0, 1, 0.1)
xtick_labels = ['%.1f'%x for x in lbl_range_x]
lbl_range_y = np.arange(-0.5, 1.5, 0.1)
ytick_labels = ['%.1f'%x for x in lbl_range_y]
plt.xticks( Nb*lbl_range_x, xtick_labels )
plt.yticks( range(0,Nrho,Nrho/len(lbl_range_y)), ytick_labels )  
plt.xlabel('b')
plt.ylabel('rho')
plt.title('U(rho) for a = 0')


# Second graph, position of the minima as b evolves
Nb = 50
a_s = np.linspace(0, 1, Nb)
b_s = np.linspace(0, 1, Nb)
rhos = np.linspace(-0.5, 1.5, Nrho)

Y = []
X = []
Z = []
C = []

for i in range(Nb):
    b = b_s[i]
    for j in range(Nb):
        a = a_s[j]
        f = lambda rho: (1.0/2.0)*(rho*((1-rho)**2 + a) - (1-rho)*(rho**2 + b))
        sol1 = root(f, 0)
        sol2 = root(f, 1)
        sol3 = root(f, 0.5)
        X.append(b)
        Y.append(a)
        Z.append(sol1.x)
        C.append('b')
        X.append(b)
        Y.append(a)
        Z.append(sol3.x)
        C.append('r')
        X.append(b)
        Y.append(a)
        Z.append(sol2.x)
        C.append('b')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(X,Y,Z, c = C, marker='.')
ax.set_xlabel('b')
ax.set_ylabel('a')
ax.set_zlabel('Roots')
plt.title('Roots of the derivative')

plt.show()