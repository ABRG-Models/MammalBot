import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy.polynomial.polynomial as poly 
from mpl_toolkits.mplot3d import Axes3D

def checkRoots( a, b, Delta ):
    if np.isnan(a) or np.isnan(b):
        return 

    X = (a + b - 2)/6.0
    Y = (a - b)/4.0
    
    dU = lambda rho,a,b: rho**3 + (a+b-2)*rho/2.0 + (a - b)/2.0
    rs = poly.polyroots([(a-b)/2.0, (a+b-2.0)/2.0,0,1.0])
    print "a,b before: ", a,b
    A = -pow(Y - np.sqrt(Delta + 0J), 1/3.0)#-pow(Y**2 -Delta, 1/3.0)/pow(Y + np.sqrt(Delta), 1/3.0)
    B = pow(Y + np.sqrt(Delta + 0J), 1/3.0)
    w = -0.5 + np.sqrt(3)*1J/2.0
    w2 = -0.5 - np.sqrt(3)*1J/2.0
    r1 = A - B 
    r2 = w2*A - w*B
    r3 = w*A - w2*B
    U = lambda rho,a,b: (1 + rho)**2*(1 - rho)**2 + a*(1+rho)**2 + b*(1 - rho)**2
    print "Heights: {:.2f}, {:.2f}, {:.2f}".format(U(r2,a,b)-U(r1,a,b), U(r3,a,b)-U(r1,a,b), U(r3,a,b)-U(r2,a,b))
    print "Roots theory: {:.2f}, {:.2f}, {:.2f}".format(r1,r2,r3)
    print "Values: {:.2f}, {:.2f}, {:.2f}".format(U(r1,a,b), U(r2,a,b), U(r3,a,b))

# Discriminant

fig = plt.figure()
ax = fig.add_subplot(1,2,1,projection='3d')

# Base space
A = np.arange(0, 2, 0.01)
B = np.arange(0, 2, 0.01)
A, B = np.meshgrid(A, B)
X = (A + B - 2)/2.0
Y = (A - B)/2.0
Z = -4*X**3 - 27*Y**2
# Z[Z<0] = -1
# Z[Z==0] = 0
# Plot the surface.
surf = ax.plot_surface(A, B, Z, cmap = cm.viridis, rstride=1, cstride=1,
                       linewidth=0, antialiased=True, vmin=-1, vmax = 1)
ax.set_xlabel('a')
ax.set_ylabel('b')

# Contours of the elliptic curve
ax = fig.add_subplot(1,2,2)
s = ax.pcolormesh(A,B,Z, cmap='RdBu', vmin=-0.2, vmax=0.1)
fig.colorbar(s, shrink=0.5, aspect=10)

l = 1.0
y, x = np.ogrid[-l:l:200j, -l:l:200j]
b = np.reshape(1 - 2*y.ravel() - 3*x.ravel(), y.shape) 
a = np.reshape(4*y.ravel() + b.ravel(), x.shape)

ax.contour(A, B, Z, [-0.2, 0], linewidths = [2.0, 2.0], colors = ['b', 'k'])
ax.set_xlabel('a')
ax.set_ylabel('b')
fig = plt.figure()
ax = fig.add_subplot(1,2,1,projection='3d')

# Base space
X = np.arange(0, 2, 0.05)
Y = np.arange(0, 2, 0.05)
X, Y = np.meshgrid(X, Y)

Z = ((X - Y)/4.0)**2 + ((X + Y - 2)/6.0)**3
# Z[Z<0] = 0
# Plot the surface.
surf = ax.plot_surface(X, Y, Z, cmap = cm.viridis, rstride=1, cstride=1,
                       linewidth=0, antialiased=True, vmin=0, vmax = 0.1)

# Elliptic mapping
ax = fig.add_subplot(1,2,2,projection='3d')
A = np.arange(0, 2, 0.05)
B = np.arange(0, 2, 0.05)
A, B = np.meshgrid(A, B)
X = -(A + B - 2)/6.0
Y = (A - B)/4.0
Z = Y**2 - X**3
# Z[Z<0] = 0
# Plot the surface.
surf = ax.plot_surface(X, Y, Z, cmap = cm.viridis, rstride=1, cstride=1,
                       linewidth=0, antialiased=True, vmin=0, vmax = 0.1)

# Elliptic curve plus a/b trajectory plus potential
fig,ax = plt.subplots(1,3)
# deltas = [ -0.01, -0.007, -0.001, 0.0, 0.01 ]
deltas = [ -0.01 ]
# deltas = np.linspace(0.0, 0.001, 4)

for i in range(len(deltas)):
    X = np.arange(0, 2, 0.05)
    Y = np.arange(0, 2, 0.05)
    X, Y = np.meshgrid(X, Y)
    Z = ((X - Y)/4.0)**2 + ((X + Y - 2)/6.0)**3
    ax[1].pcolormesh(X,Y,Z, cmap='RdBu', vmin=-0.05, vmax=0.1)

    Delta = deltas[i]
    l = 1.0
    y, x = np.ogrid[-l:l:200j, -l:l:200j]
    b = np.reshape(1 - 2*y.ravel() - 3*x.ravel(), y.shape) 
    a = np.reshape(4*y.ravel() + b.ravel(), x.shape)
    ax[0].contour(x.ravel(), y.ravel(), pow(y, 2) - pow(x, 3) - Delta, [0], linewidths=[2.0 if Delta == 0.0 else 1.0])

    
    ax[1].contour(a.ravel(), b.ravel(), pow((a-b)/4.0, 2) - pow(-(a+b-2)/6.0, 3) - Delta, [0], colors = ['k'], linewidths=[2.0 if Delta == 0.0 else 1.0])

    f = lambda x: x**3 + Delta
    map_b = lambda x,y: 1 - 2*y - 3*x
    map_a = lambda y,b: 4*y + b

    px = 0.12
    # px = np.sqrt(-Delta + 0J)
    # print "px: ", px
    py = np.sqrt(f(px))
    bp = map_b(px,py)
    ap = map_a(py,bp)
    qx = 0.22
    qy = np.sqrt(f(qx))
    bq = map_b(qx,qy)
    aq = map_a(qy,bq)
    s = (qy - py)/(qx - px)
    # b = -px*k + py 
    
    # if not np.isnan(s):
        
    
    # pol = np.poly1d([-1, k**2, 2*k*b+3, b**2-5])

    # x = np.roots(pol)
    # y = np.sqrt(f(x))
    xr = s**2 - px - qx
    yr = py + s*(xr - px)
    br = map_b(xr,yr)
    ar = map_a(yr,br)

    ax[0].plot(px, py, "ko")
    ax[0].plot(qx, qy, "ro")
    ax[0].set_xlim([-0.5, 1.5])
    ax[0].set_ylim([-1, 1])
    ax[0].plot(xr, yr, "go")
    ax[0].grid()

    ax[1].plot(ap, bp, "ko")
    ax[1].plot(aq, bq, "ro")
    ax[1].plot(ar, br, "go")
    ax[1].set_xlim([0, 2])
    ax[1].set_ylim([0, 2])
    ax[1].grid()

    U = lambda rho,a,b: (1 + rho)**2*(1 - rho)**2 + a*(1+rho)**2 + b*(1 - rho)**2
    checkRoots( ap, bp, Delta )
    checkRoots( aq, bq, Delta )
    checkRoots( ar, br, Delta )
    rho = np.linspace(-5, 5, 100)
    ax[2].plot(rho, U(rho,ap, bp), 'k')
    ax[2].plot(rho, U(rho,aq, bq), 'r')
    ax[2].plot(rho, U(rho,ar, br), 'g', linewidth = 2.0)
    ax[2].set_xlim([-2, 2])
    ax[2].set_ylim([-5,5])
    # x = np.linspace(-5, 5)
    # ax[0].plot(x, k*x+b)
    plt.pause(1)
    #ax[0].cla()
plt.grid()



fig, ax = plt.subplots(1,3)
dU = lambda rho,a,b: rho**3 + (a+b-2)*rho/2.0 + (a - b)/2.0
rho = np.linspace(-1.1, 1.1, 100)
gammas = np.linspace(-1,2,10)

plt.ion()

# for i in range(len(gammas)):
#     gamma = gammas[i]
#     a = gamma
#     b = gamma
#     r1 = 0.0
#     r2 = -np.sqrt(gamma-1 + 0J)*1J
#     r3 = np.sqrt(gamma-1 + 0J)*1J
#     ax[0].plot( rho, dU(rho,a,b))
#     ax[0].plot( r1, 0, '*') 
#     ax[0].plot( r2, 0, '*') 
#     ax[0].plot( r3, 0, '*')
#     ax[0].plot([-1, 1], [0, 0], 'k--')
#     rs = poly.polyroots([(a-b)/2.0, (a+b-2.0)/2.0,0,1.0])
    # print "Numerical: ", rs, ", theory: ", r1,r2,r3
    # plt.pause(0.01)
    # plt.cla()

c = np.linspace(1.01,2.0,100)
sqDelta = lambda c,v: ((v/2.0)**2 + ((c-1)/3.0)**3)**(1.0/2.0)
f = lambda c,v: np.cbrt(-v/2.0 + sqDelta(c,v))
h = lambda c,v: np.cbrt(-v/2.0 - sqDelta(c,v))
Deltap = lambda c,v: 0.5*(-0.5 + ((c-1)/3.0)**2)
X2 = lambda c, v: ((c-1)/6.0)**2.0
dUda = lambda c,v: (1.0/(12*sqDelta(c,v)))*(sqDelta(c,v)*(f(c,v)**2 - h(c,v)**2) - 4*Deltap(c,v)*(f(c,v)**2 + h(c,v)**2))/X2(c,v)
# f = lambda c: 1/(2*(c - 1))
# ax[0].plot( c, dUda(c,0))
dres = np.zeros(len(c))

ax[1].plot( c, dUda(c,0.1), linewidth = 2.0 )
ax[1].set_title('Shape stability, v = 0.1')
ax[1].set_xlabel('c')
ax[1].set_ylabel('dU/da')
ax[1].set_ylim([0,1000])
ax[0].plot( c, dUda(c,0.2), linewidth = 2.0 )
ax[0].set_xlabel('c')
ax[0].set_ylabel('dU/da')
ax[0].set_title('Shape stability, v = 0.2')
ax[0].set_ylim([0,1000])
ass = np.linspace( 1, 1.5, 200 )
dis0 = np.zeros(len(ass))
dis1 = np.zeros(len(ass))
dis2 = np.zeros(len(ass))
pos = np.zeros(len(ass))
b = 1.2

for i in range(len(ass)):
    a = ass[i]
    dU = lambda rho,a,b: rho**3 + (a+b-2)*rho/2.0 + (a - b)/2.0
    rs = poly.polyroots([(a-b)/2.0, (a+b-2.0)/2.0,0,1.0])
    dis0[i] = np.abs(rs[0] - rs[1])
    dis1[i] = np.abs(rs[0] - rs[2])
    dis2[i] = np.abs(rs[2] - rs[1])
    # print rs[0] if np.imag(rs[0]) == 0 else (rs[1] if np.imag(rs[1]) == 0 else rs[2])
    pos[i] = rs[0] if np.imag(rs[0]) == 0 else (rs[1] if np.imag(rs[1]) == 0 else rs[2])
# ax[2].plot(ass,dis0)
# ax[2].plot(ass,dis1)
# ax[2].plot(ass,dis2)
ax[2].plot(ass,pos)
# ax[2].plot(ass,dis1)
# ax[2].plot(ass,dis2)
ax[2].set_title('Position of the real root')

plt.ioff()
plt.show()
