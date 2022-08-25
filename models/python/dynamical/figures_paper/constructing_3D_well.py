import numpy as np
import matplotlib.pyplot as plt

# The idea is to start with a baseline critical manifold and deform
# it until it gives the required shape of the catastrophe manifold
# We start the exploration by plotting the curves along the lines u + v = k

rho1 = -1.0
rho2 = 1.0

f = lambda rho: (rho - rho1)*(rho - rho2)*(rho - (rho1 + rho2)/2.0)
u = lambda rho, k: (-2.0*f(rho) - k*(rho - rho2))/(rho2 - rho1)

rho = np.linspace(-1.0, 1.0, 100)

# Plot for a specific k
k = 1.0
plt.plot( u(rho, k), rho, 'k')
k = 0.0
plt.plot( u(rho, k), rho, 'b')
k = 2.0
plt.plot( u(rho, k), rho, 'r')
plt.title('The transformation of the 2D as you increase k')

# 3D plot
def plotSurfaceByPlanes( ax, u, rho_min = -1.0, rho_max = 1.0, k_min = -1.0, k_max = 2.6, n_lines = 50 ):
    ks = np.linspace(k_min, k_max, n_lines)
    rho = np.linspace(rho_min, rho_max, 100)

    for i in range(len(ks)):
        k= ks[i]
        ax.plot3D(u(rho, k), k - u(rho, k), rho, 'k', alpha = 0.5)

        ax.set_xlabel('u')
        ax.set_ylabel('v')
        ax.set_zlabel('rho')
        plt.title('Surface traced by the family of 2D polynomials')

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

vp = lambda rho: -2.0*(rho - rho1)*(rho - (rho1 + rho2)/2.0)
up = lambda rho: -2.0*(rho - rho2)*(rho - (rho1 + rho2)/2.0)

plotSurfaceByPlanes(ax, u)
ax.plot3D(0*rho, vp(rho), rho, 'r')
ax.plot3D( up(rho), 0*rho, rho, 'r')
ax.plot3D(0, 0, (rho1 + rho2)/2.0, 'r.', markersize = 15)

# ax.set_xlim([0, 1])
# ax.set_ylim([0, 1])
# ax.set_zlim([-1, 1])
# Plotting the 2D ridges
fig, ax = plt.subplots(1, 2)
ax[0].plot(up(rho), rho)
ax[0].plot(np.abs(f(rho)), rho, 'r')
ax[1].plot(vp(rho), rho)
ax[1].plot(np.abs(f(rho)), rho, 'r')
ax[0].axis([0, 1, -1, 1])
ax[1].axis([0, 1, -1, 1])
plt.suptitle('Contour of the surface in the coordinate planes')

fig, ax = plt.subplots(1, 3)
# Plot animation

# for i in range(len(ks)):
#     k= ks[i]
#     ax.plot( u(rho, k), rho, 'k')
#     plt.pause(0.1)


# Ploting the parabolic contours for 3 motivations
rho1 = -1.0
rho2 = -.0
rho3 = 1.5
rho = np.linspace(rho1, rho3, 100)

print(np.sqrt(-(rho1*rho2 + rho2*rho3 + rho1*rho3)/6.0))

A1 = 1.0
A2 = 0.5
A3 = 1.0
midp = lambda rho: 3.0*rho**2 - 2.0*(rho1 + rho2 + rho3)*rho + (rho1*rho2 + rho2*rho3 + rho1*rho3)
f3 = lambda rho: (rho - rho1)*(rho - rho2)*(rho - rho3)*midp(rho)
up = lambda rho: -(rho - rho2)*(rho - rho3)*midp(rho)/A1
vp = lambda rho: -(rho - rho1)*(rho - rho3)*midp(rho)/A2
wp = lambda rho: -(rho - rho1)*(rho - rho2)*midp(rho)/A3

srho = (rho1 + rho2 + rho3)
mrho = (rho1*rho2 + rho2*rho3 + rho1*rho3)
rhom1 = (srho + np.sqrt(srho**2 - 3.0*mrho))/3.0
rhom2 = (srho - np.sqrt(srho**2 - 3.0*mrho))/3.0
print(f"{rhom1} and {rhom2}")
ax[0].plot(up(rho), rho)
ax[0].plot(np.abs(f3(rho)), rho, 'k--')
ax[0].plot(up(rhom1), rhom1, 'r.', markersize = 10)
ax[0].plot(up(rhom2), rhom2, 'r.', markersize = 10)
ax[1].plot(vp(rho), rho)
ax[1].plot(np.abs(f3(rho)), rho, 'k--')
ax[1].plot(vp(rhom1), rhom1, 'r.', markersize = 10)
ax[1].plot(vp(rhom2), rhom2, 'r.', markersize = 10)
ax[2].plot(wp(rho), rho)
ax[2].plot(np.abs(f3(rho)), rho, 'k--')
ax[2].plot(wp(rhom1), rhom1, 'r.', markersize = 10)
ax[2].plot(wp(rhom2), rhom2, 'r.', markersize = 10)

ax[0].axis([0, 1, rho1, rho3])
ax[1].axis([0, 1, rho1, rho3])
ax[2].axis([0, 1, rho1, rho3])

fig, ax = plt.subplots()
sigma = 28.0
kernel = lambda rho, r, sigma: np.exp(-sigma*(rho - r)**2)
plt.plot(rho, f3(rho))
plt.plot([rho1, rho3], [0, 0], 'k--')
plt.plot(rho, kernel(rho, rho1, 3.0))
plt.plot(rho, kernel(rho, rho2, 1.0))
plt.plot(rho, kernel(rho, rho3, 3.0))

# Plotting hyperplane w = 0
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

u3 = lambda rho, k: (-f3(rho) - k*(rho - rho2))/(rho2 - rho1)
plotSurfaceByPlanes(ax, u3, rho_min = rho1, rho_max = rho3, k_min = -10.0, k_max = 10.0)
# ax.set_xlim([0, 1])
# ax.set_ylim([0, 1])
ax.set_zlim([rho1, rho3])
plt.show()
