import numpy as np  
import matplotlib.pyplot as plt 

T = 50000
# Parameters
alpha1 = 
beta1 =
# Kernels
c1 = 0.2
c2 = 0.05
eta = lambda rho: np.exp(-rho**2/(2*c1**2))
xi = lambda x: np.exp(-x**2/(2*c2**2))
# Maps
# Motivational system
fu = lambda u, v, rho, x: alpha1*(1 - u) + beta1*eta(rho + 1)*xi(x - x1)
fv = lambda u, v, rho, x: alpha2*(1 - v) + beta2*eta(rho - 1)*xi(x - x2)
frho = lambda u, v, rho: -rho**3/3.0 - (rho + 1.0)*u/2.0 - (rho - 1.0)*v/2.0 + rho
# Motor system
fphi = lambda u, v, rho:
