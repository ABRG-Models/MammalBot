# Heat - food conflict
import numpy as np
import matplotlib.pyplot as plt
import simulator4b as sm
from models import *
from fold2m import *

h = 0.005
model = sm.Model(h)
# Parameters
# The values are choosen so that the agent is in deficit
model.a1 = 1.0
model.a2 = 2.0/10.0
model.b1 = 18.0
model.b2 = 18.0
model.sigma = 8.0
model.epsilon = 0.005   

world = sm.World(h)
experiment = sm.Experiment( model, world, h )


x0 = [0.05, 0.17, 0.3]
T = 50000
experiment.world.tmin = -10.0
t, X1, pos1 = experiment.run( x0, T )

experiment.world.tmin = -2.0
t, X2, pos2 = experiment.run( x0, T )

fig = plt.figure()
gs = fig.add_gridspec(2, 2, hspace=0, wspace=0, width_ratios = [0.2, 0.8])
(ax1, ax2), (ax3, ax4) = gs.subplots(sharex='col', sharey='row')
fig.suptitle('Hunger-temperature conflict')
x = np.linspace(-1.2, 1.2, 100)
ax1.plot(experiment.world.foodSignal(x), x, 'tab:green')
ax1.plot(experiment.world.tSignal(x)/2.0 + 1.0, x, 'tab:red')
ax1.plot([0, 1], [1, 1], 'k--' )
ax1.plot([0, 1], [-1, -1], 'k--' )
ax1.axis([0, 1, -1.2, 1.2])
ax3.plot(experiment.world.foodSignal(x), x, 'tab:green')
ax3.plot(experiment.world.tSignal(x)/2.0 + 1.0, x, 'tab:red')
ax3.plot([0, 1], [1, 1], 'k--' )
ax3.plot([0, 1], [-1, -1], 'k--' )
ax3.axis([0, 1, -1.2, 1.2])
ax2.plot(t, pos1)
ax2.plot([0, t[-1]], [1, 1], 'k--')
ax2.plot([0, t[-1]], [-1, -1], 'k--')
ax2.axis([0, t[-1], -1.2, 1.2])
ax4.plot(t, pos2)
ax4.plot([0, t[-1]], [1, 1], 'k--')
ax4.plot([0, t[-1]], [-1, -1], 'k--')
ax4.axis([0, t[-1], -1.2, 1.2])

fig2 = plt.figure()
plt.hist(pos1)
plt.hist(pos2)
plt.show()
