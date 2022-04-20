import numpy as np
import matplotlib.pyplot as plt

N = 100
T = 20000

# nonlinearilties
qg= 0.950997659088
xg= 26.594968159566278
bg= 0.2818239551304378
A= 3.5505937843746906
qf= 0.8274882807912485
xf= 26.594968159566278
bf= 0.2818239551304378

# Presynaptic nonlinearity
def f(x):
    return 0.5*(2*qf-1.+np.tanh(bf*(x-xf)))
# Postsynaptic nonlinearity
def g(x):
    return 0.5*(2*qg-1.+np.tanh(bg*(x-xg)))

beta = 0.82
rm = 76.0
h0 = 2.46
tau = 10.0
# f = lambda eta: qf if xf <= eta else -(1 - qf)
# g = lambda eta: qg if xg <= eta else -(1 - qg)
phi = lambda x: rm/(1.0 + np.exp(-beta*(x - h0)))

p = 10
xi = np.random.normal(0.0, 1.0, (N,p))
dt = 0.01

ka = np.zeros(T)
kb = np.zeros(T)
kc = np.zeros(T)
ka[0] = 0.5
kb[0] = 0.4
kc[0] = 0.1

Ia = xi[:,0]
Ib = xi[:,1]
Ic = xi[:,2]
rho = np.zeros(N)

# Training
c = 0.001
J = np.zeros((N,N))

for i in range(N):
    for j in range(N):
        cij = 1 if np.random.random() < c else 0

        for k in range(p):            
            J[i,j] += (cij*A/(c*N))*f(phi(xi[i,k]))*g(phi(xi[j,k]))

print "Presenting stimuli"
# Presentation
# I = phi(Ia)
# I = Ia
ta = np.zeros(T)
tb = np.zeros(T)
tc = np.zeros(T)

alpha = 0.2
rrate = lambda x: (x > 0.0)
# rrate = 1.0
simi = lambda a, r: np.mean(np.multiply(g(phi(a)), r))

for i in range(T-1):
    # I = np.heaviside(1.-ka[i]-0.5,0)*Ia + np.heaviside(1.-kb[i]-0.5,0)*Ib + np.heaviside(1.-kc[i]-0.5,0)*Ic
    I = ka[i]*Ia + kb[i]*Ib + kc[i]*Ic
    # rho[:,i+1] = rho[:,i] + dt*(-rho[:,i] + phi(I + J.dot(rho[:,i])))/tau
    rho = rho + dt*(-rho + phi(I + J.dot(rho)))/tau
    ta[i+1] = simi(Ia, rho)
    tb[i+1] = simi(Ib, rho)
    tc[i+1] = simi(Ic, rho)
    max_ac = np.argmax(np.array([ta[i+1], tb[i+1], tc[i+1]]))
    ka[i+1] = ka[i] + dt*(-alpha*ka[i] + rrate(ta[i+1]))
    kb[i+1] = kb[i] + dt*(-alpha*kb[i] + rrate(tb[i+1]))
    kc[i+1]= kc[i] + dt*(-alpha*kc[i] + rrate(tc[i+1]))

    if ka[i+1] > 1.0:
        ka[i+1] = 1.0
    if kb[i+1] > 1.0:
        kb[i+1] = 1.0
    if kc[i+1] > 1.0:
        kc[i+1] = 1.0



# simi = lambda a, b: a.dot(b)/(np.linalg.norm(a)*np.linalg.norm(b))

# pa = xi[:,0]#phi(xi[:,0])
# pb = xi[:,1]#phi(xi[:,1])
# pc = xi[:,2]#phi(xi[:,2])
# print( "similar a: ", simi(pa, rho[:,-1]) )
# print( "similar b: ", simi(pb, rho[:,-1]) )
# print( "similar c: ", simi(pc, rho[:,-1]) )

# fig = plt.figure()
# plt.imshow(J)
fig, ax = plt.subplots(2,1)
xs = np.arange(N)
ax[0].plot(ta, label="Tendency a")
ax[0].plot(tb, label="Tendency b")
ax[0].plot(tc, label="Tendency b")
ax[1].plot(ka, label="Drive a")
ax[1].plot(kb, label="Drive b" )
ax[1].plot(kc, label="Drive c")
ax[0].legend()
ax[1].legend()
plt.show()

