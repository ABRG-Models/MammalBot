# Heat - food conflict
from turtle import pos
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
model.b1 = 5.0
model.b2 = 5.0
model.sigma = 10.0
model.epsilon = 0.005   

world = sm.World(h)
experiment = sm.Experiment( model, world, h )

Npos = 50
positions = np.linspace(0.8, 2.0, Npos)
lifespans = np.zeros_like(positions)
T = 300000
Ypos = np.zeros((Npos, T))

doExp = False

if doExp:

    for i in range(len(positions)):

        experiment.world.food = -positions[i]
        experiment.world.water = positions[i]
        x0 = [1.0, 0.0, -1.0]
        
        t, X, pos = experiment.run( x0, T )
        
        Xm = X[2,:]
        life = np.where(np.abs(Xm) > 0.1 )
        print("i = %d"%i)
        lifespans[i] = life[0][-1]
        Ypos[i, :] = pos

    np.save("lifespans.npy", lifespans)
    np.save("Ypos.npy", Ypos)
else:
    lifespans = np.load("lifespans.npy")
    Ypos = np.load("Ypos.npy")
    print(Ypos.shape)
fig = plt.figure()
plt.plot(2*positions, lifespans/float(T), linewidth = 2.5)
plt.xlabel("Separation")
plt.ylabel("Lifespan")

fig = plt.figure()
H = np.zeros((Npos, 60))

for i in range(Npos):
    counts, bins, patches = plt.hist(Ypos[i, :], 60, density=True)
    H[i,:] = counts
plt.imshow(H)
plt.show()
