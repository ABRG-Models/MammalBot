import numpy as np
import matplotlib.pyplot as plt
from fold2m import *

fig, ax = plt.subplots(1,1)
sigma = 10.0
a = 2.0
b = 60.0
print(b/(a + b))
p = lambda v: (2.0 - np.sqrt(-(1.0/sigma)*np.log(a*v/(b*(1.0 - v)))))**2
u = lambda v: a*np.exp(-sigma*p(v))/(a + b*np.exp(-sigma*p(v)))
# plot_fold_projection( ax )
v = np.linspace(0,0.99,100)
print(u(v))
plt.plot(u(v), v)
plt.xlabel('u')
plt.ylabel('v')
plt.show()