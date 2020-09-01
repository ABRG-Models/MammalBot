import numpy as np
import matplotlib.pyplot as plt
import numpy.polynomial.polynomial as poly 

def add_embed_potential( x, y, a, b, low = 0.06, high = 0.2 ):
    U = lambda rho, a, b: rho**2*(1 - rho)**2 + a*rho**2 + b*(1 - rho)**2

    fig = plt.gcf()
    sa1 = fig.add_axes( [x, y, .08, .08] )
    sa1.set_xticks([])
    sa1.set_yticks([])

    rho = np.linspace( -0.1, 1.1, 100 )
    sa1.plot( rho, U(rho, a, b), linewidth = 2.0)
    sa1.plot( [0.5, 0.5], [low, high], 'k--')
    sa1.patch.set_alpha(0.7)
    sa1.set_ylim( low , high )


def nroots( a, b ):
    r = poly.polyroots([-b, a+b+1,-3,2])
    
    diff_r = []

    for i in range(r.size):
        b = False
        # if np.abs(np.imag(r[i])) < 0.01:
        #     print r[i]

        for j in range(len(diff_r)):
            if np.abs(np.real(r[i]) - np.real(diff_r[j])) < 0.1:
                b = True
        
        if not b: #and np.abs(np.imag(r[i])) < 0.01:
            diff_r.append(r[i])

    if len(diff_r) == 2:
        print r

    return len(diff_r)

N = 100
a = np.linspace( 0, 1, N )
b = np.linspace( 0, 1, N )
X = np.zeros((N,N))

for i in range( N ):
    for j in range( N ):
        X[i,j] = nroots( a[i], b[j] )

lbl_range = np.arange(0, 1, 0.1)
xtick_labels = ['%.1f'%x for x in lbl_range]
plt.imshow( X, interpolation='nearest', origin='lower')
plt.colorbar()
plt.xlabel( 'a' )
plt.xticks( N*lbl_range, xtick_labels ) 
plt.ylabel( 'b' )
plt.yticks( N*lbl_range, xtick_labels )
plt.title( 'Number of roots of the polynomial' )

# Adding subaxis
add_embed_potential( 0.15, 0.13, 0.1, 0.08 )
add_embed_potential( 0.15, 0.45, 0.1, 0.25)
add_embed_potential( 0.45, 0.15, 0.25, 0.1)
add_embed_potential( 0.40, 0.45, 0.4, 0.4, high = 0.5)
add_embed_potential( 0.5, 0.55, 0.48, 0.5, high = 0.5)

plt.show()