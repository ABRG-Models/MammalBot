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
model.a2 = 1.0
model.b1 = 18.0
model.b2 = 18.0
model.sigma = 8.0
model.epsilon = 0.005   

world = sm.World(h)
experiment = sm.Experiment( model, world, h )


x0 = [0.05, 0.17, 0.3]
T = 100000
t, X, pos = experiment.run( x0, T )

plt.plot(t, pos)
plt.show()
