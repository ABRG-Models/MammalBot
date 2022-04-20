# Figure 1a. The critical manifold plus the theoretical flow and closed orbit
from mayavi import mlab
import matplotlib.pyplot as plt  
from scipy.optimize import fsolve
import numpy as np 
from models import *

mlab.figure(size = (1024,768),\
            bgcolor = (1,1,1), fgcolor = (0.5, 0.5, 0.5))
mlab.clf()
# Drawing the critical manifold
rho1 = -1.0
rho2 = 1.0

u, v, rho = np.mgrid[0:1:50j, 0:1:50j, -1.0:1.0:50j]
# Critical manifold
values = ((rho - rho1)*(rho - rho2)*(rho - (rho1+rho2)/2.0) + 
                    (1-u)*(rho - rho1)/2.0 + (1-v)*(rho - rho2)/2.0)
mlab.contour3d(u, v, rho, values, contours=[0], opacity = 0.8, color = (93.0/255.0, 139.0/255.0, 244.0/255.0), transparent = True)

# Plotting the trajectory
a = 2.0
b = 65.0
sigma = 10.0

# Integrating model and plotting closed trajectory
model = TwoMotivations()
model.b1 = model.b2 = b
model.a1 = model.a2 = a
model.sigma = sigma
model.epsilon = 0.01

u0 = 0.75
t,X = model.integrate(T = 20000, q0 = [u0, u0, np.sqrt(u0)])
mlab.plot3d(X[0,1300:], X[1,1300:], X[2,1300:], tube_radius = 0.007, color = (0.0, 0.0, 0.0), line_width = 1.0)

# Arrows on the trajectory
def draw_arrow_on_trajectory(t1, s, dt, mode = 'arrow'):
    x = X[0,t1]
    y = X[1,t1]
    z = X[2,t1]
    u = X[0,t1+dt]-X[0,t1]
    v = X[1,t1+dt]-X[1,t1]
    w = X[2,t1+dt]-X[2,t1]
    mlab.quiver3d(x, y, z, u, v, w, line_width=3, scale_factor=s, color = (0.0, 0.0, 0.0), mode = mode)

draw_arrow_on_trajectory(1500, 2, 1000)
draw_arrow_on_trajectory(5750, 2, 185, mode = '2darrow')
draw_arrow_on_trajectory(5780, 2, 150, mode = '2darrow')
draw_arrow_on_trajectory(6260, 3, 100)
draw_arrow_on_trajectory(8200, 2, 1000)
draw_arrow_on_trajectory(12560, 2, 185,mode = '2darrow')
draw_arrow_on_trajectory(12590, 2, 150,mode = '2darrow')
draw_arrow_on_trajectory(13090, 3, 100)

# Field
# u0 = 0.2
# v0 = 0.0
# t,X = model.integrate(T = 50000, q0 = [u0, v0, np.sqrt(u0)])
# mlab.plot3d(X[0,:], X[1,:], X[2,:], tube_radius = None, color = (241.0/255.0, 208.0/255.0, 100.0/255.0), line_width = 2.0)

# Plotting fold
yf = lambda x: np.sqrt(2.0*(x)**3/27.0)
rho_f = lambda x: np.sqrt(x/6.0)

x = np.linspace(0, 1.5, 100)
y = yf(x)
u = (x + y)/2.0
v = (x - y)/2.0
mlab.plot3d(u, v, -rho_f(u + v), tube_radius = None, color = (241.0/255.0, 208.0/255.0, 100.0/255.0), line_width = 2.0)
y = -yf(x)
u = (x + y)/2.0
v = (x - y)/2.0
mlab.plot3d(u, v, rho_f(u + v), tube_radius = None, color = (241.0/255.0, 208.0/255.0, 100.0/255.0), line_width = 2.0)

# field
g1 = lambda u, v, rho: -a*u + b*(1 - u)*np.exp(-sigma*(rho + 1.0)**2)
g2 = lambda u, v, rho: -a*v + b*(1 - v)*np.exp(-sigma*(rho - 1.0)**2)
f = lambda u, v, rho: rho**3 - (u + v)*rho/2.0 - (u - v)/2.0
us = np.linspace(0, 1, 10)
vs = np.linspace(0, 1, 10)
s = 0.05
ffold = lambda u, v: 2.0*(u + v)**3 - 27.0*(u - v)**2
for i in range(len(us)):
    for j in range(len(vs)):
        if( ffold(us[i], vs[j])) < 0:
            continue
        fk = lambda rho: f(us[i], vs[j], rho)
        rho = fsolve( fk, 1.0)
        df = np.array([g1(us[i], vs[j], rho),g2(us[i], vs[j], rho),f(us[i], vs[j], rho)])
        df = df/np.linalg.norm(df)
        mlab.quiver3d(us[i], vs[j], rho, \
                        s*df[0], s*df[1], s*df[2], \
                        line_width=3, scale_factor=1.0, color = (1.0, 1.0, 1.0), opacity = 0.4)

for i in range(len(us)):
    for j in range(len(vs)):
        if( ffold(us[i], vs[j])) < 0:
            continue
        fk = lambda rho: f(us[i], vs[j], rho)
        rho = fsolve( fk, -1.0)
        df = np.array([g1(us[i], vs[j], rho),g2(us[i], vs[j], rho),f(us[i], vs[j], rho)])
        df = df/np.linalg.norm(df)
        mlab.quiver3d(us[i], vs[j], rho, \
                        s*df[0], s*df[1], s*df[2], \
                        line_width=3, scale_factor=1.0, color = (1.0, 1.0, 1.0), opacity = 0.4)


# mlab.xlabel('u')
# mlab.ylabel('v')
# mlab.zlabel('rho')
# mlab.axes(extent = [-.2, 1.2, -.2, 1.2, -1.2, 1.2], nb_labels = 1) 
# mlab.orientation_axes(xlabel = 'u', ylabel = 'v', zlabel = 'rho')


mlab.show()