P = lambda u, v: (1.0 - (u + v))/4.0
Q = lambda u, v: -(v - u)/8.0

Delta = lambda u, v: (Q(u,v)/2.0)**2 + (P(u,v)/3.0)**3
A = lambda u,v: (-Q(u,v)/2.0 + np.sqrt(Delta(u,v) + 0j))**(1.0/3.0)
B = lambda u,v: (-Q(u,v)/2.0 - np.sqrt(Delta(u,v) + 0j))**(1.0/3.0)
w1 =(-1.0/2.0 + np.sqrt(-3.0 + 0j)/2.0)
w2 = (-1.0/2.0 - np.sqrt(-3.0 + 0j)/2.0)

c1 = lambda rho, u, v: np.real(rho - A(u, v) - B(u, v))
c2 = lambda rho, u, v: np.real(rho - w1*A(u, v) - w2*B(u, v))
c3 = lambda rho, u, v: np.real(rho - w2*A(u, v) - w1*B(u, v))