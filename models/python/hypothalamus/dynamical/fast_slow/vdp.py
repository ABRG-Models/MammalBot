import numpy as np 
import matplotlib.pyplot as plt 

# fig, ax = plt.subplots(1,2)
# plt.ion()
plt.show(block = False)

T = 20000
epsilon = 0.05
h = 0.01
l = 0.99361
c0 = lambda x: x**3/3.0 - x
f = lambda x, y: y - x**3/3.0 + x
g = lambda x, y: epsilon*(l-x)

x = np.zeros(T)
y = np.zeros(T)
time = np.zeros(T)

x[0] = 0.5
y[0] = 0.4
for i in range(T-1):
    x[i+1] = x[i] + h*f(x[i], y[i])
    y[i+1] = y[i] + h*g(x[i], y[i])
    time[i+1] = time[i] + h


xx = np.linspace(-2, 2, 100)
plt.plot(xx, c0(xx), color = [0.5, 0.5, 0.5], linewidth = 3.0)
plt.xlabel('x')
plt.xlabel('y')
plt.plot( x, y )
plt.show()

