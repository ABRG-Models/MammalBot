import numpy as np 
import matplotlib.pyplot as plt

a = 0.
b = 0.
rhos = np.linspace(-1.1,1.1,100)
U1 = lambda rho, a, b: (rho - 1)**2*(rho + 1)**2 + a*(rho + 1)**2 + b*(rho - 1)**2

plt.figure()
s = U1(rhos,a ,b)
M = s[50]
m = np.amax(-s)
print( M+m )
plt.plot( rhos, s)
plt.show()