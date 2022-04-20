## Illustration of different kinds of singularities
import numpy as np 
import matplotlib.pyplot as plt 

fig, ax = plt.subplots(1,3)

T = 64000
e = 0.01
h = 0.001
plt.ion()
plt.show(block = False)

# Fold flow and bifurcation
f = lambda x,y: y - x**2
g = lambda x,y: -1.0*e

yc = lambda x: x**2
xc = np.linspace(-1, 1, 100)
ax[0].plot(xc, yc(xc), 'b--')

x = np.zeros(T)
y = np.zeros(T)
x[0] = 1.0
y[0] = 0.5

for i in range(1,T):
    
    x[i] = x[i-1] + h*f(x[i-1], y[i-1])
    y[i] = y[i-1] + h*g(x[i-1], y[i-1])    
    # ax[0].plot( x[(i-1):i+1], y[(i-1):i+1], 'k' )
    # plt.pause(0.1)

ax[0].plot( x, y, 'k' )
plt.ioff()
plt.show()


