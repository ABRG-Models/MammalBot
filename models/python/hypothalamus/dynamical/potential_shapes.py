import numpy as np 
import matplotlib.pyplot as plt

a = 1.
b = 0.95
rhos = np.linspace(-1.1,1.1,100)
U1 = lambda rho, a, b: (rho - 1)**2*(rho + 1)**2 + a*(rho + 1)**2 + b*(rho - 1)**2

plt.figure()
plt.plot( rhos, U1(rhos,a ,b))
plt.show()