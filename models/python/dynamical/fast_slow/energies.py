
import matplotlib.pyplot as plt  
import numpy as np 


u = 0.0
v = 0.1
w = 0.0
U2 = lambda rho: ((rho + 1)**2*(rho - 1)**2 + u*(rho + 1)**2 + v*(rho - 1)**2)/4.0
dU2 = lambda rho: rho**3 + (rho + 1.0)*u/2.0 + (rho - 1.0)*v/2.0 - rho

U3 = lambda rho: (rho + 1)**2*rho**2*(rho - 1)**2 + u*(rho + 1)**2 + v*rho**2 + w*(rho - 1)**2
dU3 = lambda rho: 2.0*(rho**3 - rho)*(3.0*rho**2 - 1) + 2*u*(rho + 1) + 2*v*rho + 2*w*(rho - 1)

rhos = np.linspace(-1.5, 1.5, 100)
fig, ax = plt.subplots( 1, 2 )

ax[0].plot( rhos, U2(rhos), 'k' )
ax[0].plot( rhos, dU2(rhos), 'k--')
ax[0].plot( [-1.5, 1.5], [0.0, 0.0], color = [0.5, 0.5, 0.5])


ax[1].plot( rhos, U3(rhos), 'k' )
ax[1].plot( rhos, dU3(rhos), 'k--')
ax[1].plot( [-1.5, 1.5], [0.0, 0.0], color = [0.5, 0.5, 0.5])
plt.show()
