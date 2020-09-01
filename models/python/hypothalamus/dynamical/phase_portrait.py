import numpy as np
import matplotlib.pyplot as plt 

def motivation_map( rho, a, b ):
    L = 10.0

    U = lambda rho: (1.0/4.0)*rho**2*(1 - rho)**2 + a*rho**2 + b*(1 - rho)**2
    dU = lambda rho: (1.0/2.0)*(rho*((1-rho)**2 + a) - (1-rho)*(rho**2 + b))

    #noise = np.random.normal(loc = 0.0, scale=self.diff_heat)/np.sqrt(h)
    dRho = -L*dU( rho ) #+ noise

    return dRho

def evolve( T, rho0, a, b ):
    h = 0.01
    N = int(T/h)
    rho = rho0

    for i in range(N):
        rho += h*motivation_map( rho, a, b )

    return rho

T = 10.0
N = 100
a = np.linspace( 0, 1, N )
b = np.linspace( 0, 1, N )
rho0 = 0.5
X = np.zeros((N,N))

for i in range( N ):
    for j in range( N ):
        X[i,j] = evolve(T, rho0, a[i], b[j] )

lbl_range = np.arange(0, 1, 0.1)
xtick_labels = ['%.1f'%x for x in lbl_range]
plt.imshow( X, interpolation='nearest', origin='lower')
plt.colorbar()
plt.xlabel( 'a' )
plt.xticks( N*lbl_range, xtick_labels ) 
plt.ylabel( 'b' )
plt.yticks( N*lbl_range, xtick_labels )
plt.title( 'Motivational stationary states, rho0 = %.1f'%rho0 )
plt.show()