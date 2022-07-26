# Dominance - dominance boundary
from nose import run
import numpy as np
import matplotlib.pyplot as plt
import simulator4a as sm
from sklearn.linear_model import Perceptron


h = 0.005
model = sm.Model(h)
# Parameters
# The values are choosen so that the agent is in deficit
model.a1 = 1.0
model.a2 = 1.0
model.b1 = 18.0
model.b2 = 18.0
model.sigma = 8.0
model.epsilon = 0.005   

world = sm.World(h)
experiment = sm.Experiment( model, world, h )

def plot_fold_projection( ax, color = 'k', offset = 0 ):
    U = np.arange(0, 1, 0.01)
    V = np.arange(0, 1, 0.01)
    U, V = np.meshgrid(U, V)
    X = U + V
    Y = U - V
    Z = 2.0*X**3 - 27.0*Y**2 - offset
    ax.contour(1-U, 1-V, Z, [0], linewidths = [2.0, 2.0], colors = [color])


x0 = [0.05, 0.17, 0.3]
T = 400000
t, X = experiment.run( x0, T )

fig, ax = plt.subplots(1, 1)

plot_fold_projection( ax, color =(0.7, 0.7, 0.7))
plot_fold_projection( ax, color =(0.7, 0.7, 0.9), offset = -0.1)
plot_fold_projection( ax, color =(0.7, 0.7, 0.9), offset = 0.1)


ax.plot(1-X[0,:], 1-X[1,:], color = [0.8, 0.8, 0.8])
ax.plot(1-X[0,experiment.interruptions], 1-X[1,experiment.interruptions], 'k', linewidth = 2.0)
ax.plot(1-X[0,0], 1-X[1,0], 'k.', markersize = 15.0)
for i in range(len(experiment.interruptions)-1):
    if X[2, experiment.interruptions[i]]*X[2, experiment.interruptions[i+1]] > 0:
        if X[2, experiment.interruptions[i]] > 0:
            ax.plot(1 - X[0, experiment.interruptions[i]], 1 - X[1, experiment.interruptions[i]], 'k.', markersize = 13.0)
        else:
            ax.plot(1 - X[0, experiment.interruptions[i]], 1 - X[1, experiment.interruptions[i]], 'r.', markersize = 13.0)
    else:
        if X[2, experiment.interruptions[i]] > 0:
            ax.plot(1 - X[0, experiment.interruptions[i]], 1 - X[1, experiment.interruptions[i]], 'k^', markersize = 15.0)
        else:
            ax.plot(1 - X[0, experiment.interruptions[i]], 1 - X[1, experiment.interruptions[i]], 'r^', markersize = 15.0)
# ax.plot(t, X[2,:])
ax.set_xlabel('Deficit u', fontsize = 16.0)
ax.set_ylabel('Deficit v', fontsize = 16.0)
plt.xticks(fontsize = 14.0)
plt.show()
