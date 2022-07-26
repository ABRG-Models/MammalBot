from telnetlib import NOP
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize

b = 55.0
N = 500
M = 500
sigmas = np.linspace(0, 10, N)
ratios = np.linspace(0, 1, M)
roots = np.zeros((N, M))

for i in range(len(sigmas)):
    for j in range(len(ratios)):
        sigma = sigmas[i]
        
        a = ratios[j]*b
        u = lambda rho: b*np.exp(-sigma*(rho + 1)**2)/(a + b*np.exp(-sigma*(rho + 1)**2))
        v = lambda rho: b*np.exp(-sigma*(rho - 1)**2)/(a + b*np.exp(-sigma*(rho - 1)**2))
        f = lambda rho: rho**3 - (u(rho) + v(rho))*rho/2.0 - (u(rho) - v(rho))/2.0

        nroots = 1
        
        try:
            sol = optimize.bisect(f, 1e-1, 2, xtol = 1e-10)
            nroots += 1               
            sol = optimize.bisect(f, -1, -1e-2, xtol = 1e-10)
            # if sol.converged:
            nroots += 1
        except:
            pass        

        roots[i, j] = nroots


plt.imshow(roots, origin='lower', extent = [-1, 1, -1, 1], cmap = 'binary')
plt.xticks([-1, -0.75,-0.25,0.25,0.75, 1], labels=['%.1f'%x for x in np.linspace(0,1,6)], fontsize=18)
plt.yticks([-1, -0.75,-0.25,0.25,0.75, 1], labels=['%.1f'%x for x in np.linspace(0,10,6)], fontsize=18)
plt.ylabel('$\sigma$',fontsize=22)
plt.xlabel('a/b ratio',fontsize=22)

cbar = plt.colorbar()
cbar.ax.tick_params(labelsize=18) 
plt.show()