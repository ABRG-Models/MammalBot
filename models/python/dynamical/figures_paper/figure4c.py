# Heat - food conflict
import numpy as np
import matplotlib.pyplot as plt
import simulator4c as sm
from models import *
from fold2m import *

h = 0.005
model = sm.Model(h)
# Parameters
# The values are choosen so that the agent is in deficit
model.a1 = 2.0
model.a2 = 2.0
model.b1 = 10.0
model.b2 = 10.0
model.sigma = 10.0
model.epsilon = 0.005   

world = sm.World(h)
experiment = sm.Experiment( model, world, h )

experiment.world.food = -1.8
experiment.world.water = 1.8
x0 = [1.0, 0.0, -1.0]
T = 300000
t, X, pos = experiment.run( x0, T )

fig, ax = plt.subplots(1, 2)
ax[0].plot(t, pos)
ax[1].plot(t, X[2,:])
plt.show()
