from mayavi import mlab
from models import *
from critical2m import *

model = TwoMotivations()
model.b1 = 97.0
model.b2 = 97.0
# model.b1 = model.b2 = 20.0
model.a1 = 2.0
model.a2 = 2.0
t,X = model.integrate(T = 100000, x0 = [0.2, 0.2, 1.0])

h = lambda vd, v: (-1.0/model.sigma)*(np.log(vd + model.a1*v) - np.log(model.b1))
u = X[0,:]
v = X[1,:]
ud = -model.a1*u + model.b1*(1.0-u)*np.exp(-model.sigma*(X[2,:] + 1)**2)
vd = -model.a1*v + model.b1*(1.0-v)*np.exp(-model.sigma*(X[2,:] - 1)**2)

invariant = np.sqrt(h(ud,u)) + np.sqrt(h(vd, v))#(ud + model.a1*u)/(vd + model.a1*v)

draw_critical2m(False)
mlab.plot3d(X[0, :], X[1,:], X[2,:], tube_radius = 0.01, line_width = 0.5, color = (1.0, 1.0, 1.0))
mlab.show()

# plt.plot(t,invariant)
a = -(X[0,:]+X[1,:])/2.0
b = -(X[0,:]-X[1,:])/2.0
ad = -(ud + vd)/2.0
bd = -(ud - vd)/2.0
plt.plot(t,b/np.sqrt(1.0 + a**2), 'k')
plt.plot(t, np.arctan(-(1.0/a)), 'r')
# plt.plot(t, ad/(1.0 + a**2))

# plt.plot(t, (bd - a*b*ad)/np.sqrt(1 + a**2))
plt.axis([t[0], t[-1], -1, 1])

# plt.plot([t[0], t[-1]], [np.exp(-4.0*model.sigma), np.exp(-4.0*model.sigma)], 'k--')
plt.title('Conservation law')
plt.xlabel('time')
# plt.axis([0, t[-1], 0, np.exp(-4.0*model.sigma)*2.0])
plt.show()