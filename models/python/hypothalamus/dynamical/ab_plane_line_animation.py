import numpy as np
import matplotlib.pyplot as plt
import numpy.polynomial.polynomial as poly 

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


plt.show()