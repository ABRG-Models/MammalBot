from PyDSTool import *
import matplotlib.pyplot as plt 
from time import clock

# Declare parameters
a = Par(1.0, 'a')
b = Par(0.0, 'b')
epsilon = Par(1.0, 'epsilon')
sigma = Par(10.0, 'sigma')

# Initial conditions
u0 = 0.0
v0 = 1.0
rho0 = 0.4

# Symbolic variables
u = Var('u')
v = Var('v')
rho = Var('rho')

# Symbolic definitions
f = epsilon*(-a*u + b*(1 - u)*Exp(-sigma*rho**2))
g = epsilon*(-a*v + b*(1 - v)*Exp(-sigma*(rho-1.0)**2))
dphi = -4.0*(rho*(rho - 1)*(2*rho - 1) + (1-u)*rho/2.0 + (1-v)*(rho-1)/2.0)

# Generator
DSargs = args(name = 'Motivational Dynamica System')
DSargs.pars = [a, b, epsilon, sigma]
DSargs.varspecs = args(u = f,
                       v = g,
                       rho = dphi)

DSargs.ics = args(u = u0, v = v0, rho = rho0 )
DSargs.tdomain = [0, 500]

ode = Generator.Vode_ODEsystem(DSargs)

# traj = ode.compute('polarization')
# pts = traj.sample(dt = 0.1)

# plt.plot(pts['t'], pts['v'])
# plt.plot(pts['t'], pts['u'])
# plt.show()
# Biffurcations
PC = ContClass(ode)

PCargs = args(name = 'EQ1', type = 'EP-C')
PCargs.freepars = ['a']
PCargs.StepSize = 1e-2
# PCargs.initdirec = pts[1]
PCargs.MaxNumPoints = 50
PCargs.MaxStepSize = 1e-1
PCargs.LocBifPoints = 'all'
PCargs.SaveEigen = True
PCargs.verbosity = 2
PC.newCurve(PCargs)

print("Computing curve...")
start = clock()
PC['EQ1'].forward()
print("Done in %.3f secs!" % (clock()-start))


PC['EQ1'].info()

PCargs.name = 'HO1'
PCargs.type = 'H-C2'
PCargs.initpoint = 'EQ1:P1'
PCargs.freepars = ['a', 'b']
PCargs.MaxNumPoints = 50
PCargs.MaxStepSize = 1e-1
PCargs.LocBifPoints = ['ZH']
PCargs.SaveEigen = True
PC.newCurve(PCargs)

print("Computing hopf curve...")
start = clock()
PC['HO1'].forward()
print("Done in %.3f secs!" % (clock()-start))

PC['HO1'].info()


PCargs.name = 'FO1'
PCargs.type = 'LP-C'
PCargs.freepars = ['a','b']
PCargs.StepSize = 1e-2
PCargs.initpoint = 'HO1:P2'
# PCargs.initdirec = pts[1]
PCargs.MaxNumPoints = 25
PCargs.MaxStepSize = 1e-1
PCargs.LocBifPoints = 'all'
PCargs.SaveEigen = True
PCargs.verbosity = 2
PC.newCurve(PCargs)

print("Computing hopf curve...")
start = clock()
PC['FO1'].forward()
print("Done in %.3f secs!" % (clock()-start))


PC.display(('a', 'b'), stability = True, figure = 1)
plt.title('Biffurcation diagram of equilibria in (a,b)')
plt.show()