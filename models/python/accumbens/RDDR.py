class RDDR:

	def __init__( self, n, m, eta = 0.1 ):
		self.s = np.zeros(m) # Outputs
		self.c = np.zeros(n) # Inputs
		self.w = np.zeros((m,n)) # Feedforward connections 
		self.a = np.zeros((m,m)) # Lateral connections

		self.eta = eta

	def evolve( self, r ):

		for i in range(self.m):
			s[i] = s[i] + np.dot( w[i,:] ,c ) + np.dot( a[i,:], s )


		for i in range( m ):
			for j in range( n ):
				w[i,j] = w[i,j] + self.eta*r*(s[i]*c[j] - w[i,j]*s[i]**2)

		for i in range(m):
			for j in range(m):
				if i != j
					a[i,j] = a[i,j] - self.eta*(s[i]*s[j] + a[i,j]*s[i]**2)