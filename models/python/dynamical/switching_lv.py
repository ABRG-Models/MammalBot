import numpy as np
import matplotlib.pyplot as plt 

c = 0.2
g = lambda x: 1 if x > 1.0 else 0
R = 0.005
eta1 = lambda w, rho: -R*g(rho)*np.heaviside(w, 0)
eta2 = lambda w, rho: -R*g(rho)*np.heaviside(w, 0)

fa = lambda a, b, w1: w1*(a - a*b)
fb = lambda a, b, w2: -w2*(b - a*b)

def evolve( T, a0, b0, w1_0, w2_0 ):
    h = 0.01
    N = int(T/h)    
    t = np.zeros(N)    
    a = np.zeros(N)
    b = np.zeros(N)
    w1 = np.zeros(N)
    w2 = np.zeros(N)
    t[0] = 0
    a[0] = a0
    b[0] = b0
    w1[0] = w1_0
    w2[0] = w2_0

    for i in range(N-1):
        a[i+1] = a[i] + h*fa( a[i], b[i], w1[i] )
        b[i+1] = b[i] + h*fb( a[i], b[i], w2[i] )
        
        w1[i+1] = w1[i] + h*eta1( w1[i],a[i] )
        w2[i+1] = w2[i] + h*eta2( w2[i], b[i] )
        t[i+1] = t[i] + h

    return a, b, w1, w2, t

a0 = 1.0
b0 = 0.1
w1_0 = 0.3
w2_0 = 0.4

a, b, w1, w2, t = evolve( 500, a0, b0, w1_0,w2_0 )

fig, ax = plt.subplots(1, 3)

ax[0].plot( a, b, '.-' )
ax[0].set_xlabel('motivation1')
ax[0].set_ylabel('motivation2')
plt.grid()
ax[1].plot(t, a)
ax[1].plot(t, b)
ax[1].set_xlabel('time')
ax[1].set_ylabel('motivation')
ax[2].plot(w1, w2)
ax[2].set_xlabel('a')
ax[2].set_ylabel('b')
plt.show()
