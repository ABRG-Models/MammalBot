import numpy as np 
import matplotlib.pyplot as plt 

sigma = 0.5
x0 = 0.0
N = lambda x: 2.0/(1 + np.exp(-sigma*(x - x0))) - 1.0
U = lambda x: x**3

x = np.linspace( -10, 10, 100 )

fig, ax = plt.subplots(	1, 3 )

ax[0].plot( x, x, linewidth = 2.0 )
ax[0].set_ylabel('Drive')
ax[0].set_xlabel('State variable')
ax[0].set_title('Linear')

ax[1].plot( x, N(x), linewidth = 2. )
ax[1].set_ylabel('Drive')
ax[1].set_xlabel('State variable')
ax[1].set_title('Normalization')

ax[2].plot( x, U(N(x)), linewidth = 2. )
ax[2].set_xlabel('State variable')
ax[2].set_ylabel('Drive')
ax[2].set_title('Urgency + normalization')
#plt.plot( x, N(x) )
plt.show()

