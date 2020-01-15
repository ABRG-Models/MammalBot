class HomeostaticSystemMeta(type):
	instance = None 

	def __new__(metaname, classname, baseclasses, attrs):
        print 'New called with'
        print 'metaname', metaname
        print 'classname', classname
        print 'baseclasses', baseclasses
        print 'attrs', attrs
        attrs['getdata'] = a.__dict__['getd3']
        # attrs['getdata'] = a.getd3
        return type.__new__(metaname, classname, baseclasses, attrs)
 
    def __init__(classobject, classname, baseclasses, attrs):
        

class HomeostaticSystem( object ):
	
	def dmap( self, u, t ):
		Tl = self.getSensorData( 'T_left' )
		Tr = self.getSensorData( 'T_right' )
		Fr = self.getSensorData( 'F_left' )
		Fl = self.getSensorData( 'F_right' )

		Ta = (Tl + Tr)/2.0
		# State variables
		theta = u[2]
		Tb = u[3]
		E = u[4]
		# Drives
		dHeat = np.abs(Tb - self.Tp)/np.abs(40.0 - 35.0)
		dFood = np.heaviside(1 - E, 0.0)*(1 - E)
		# Parameters
		mu = 5*np.linalg.norm(np.array([dHeat, dFood]))
		print('Drive heat: {}, Drive food: {}'.format(dHeat, dFood))
		k2 = 1.0
		Tc = Tb # No contact
		alpha = 0.5
		# nonlinearity
		sigma = 0.1
		f = lambda x : 2*x/15.0 if abs(x)<15 else 2*np.sign(x)
		# Diff Equations	
		dx = mu*np.array([np.cos(theta), np.sin(theta)])
		dTb = self.G - self.k1*(Tb - Ta)*self.A - k2*(1 - self.A)*(Tb - Tc)
		dE = -alpha*self.G + self.F
		
		if( dHeat > dFood ):
			print('Temperature drives: {}, {}'.format((Tb - self.Tp),f((Tb - self.Tp))))
			print('Tl: {}, Tr: {}'.format(Tl, Tr))
			dTheta = f((Tb - self.Tp)*(Tl - Tr)) + np.random.rand()
		else:
			print('Energy drives')
			print('Fl: {}, Fr: {}'.format(Fl, Fr))
			dTheta = (E - 0.9)*(Fl - Fr)

		return np.array([dx[0], dx[1], dTheta, dTb, dE ])


	def step( self, c_step, h, t ):
		dr = self.dmap( self.state[:, c_step], t )
		self.state[:, c_step + 1] = self.state[:, c_step] + h*dr
		self.state[4] = np.heaviside(self.state[4], 0.5)*self.state[4]
		# moving/ acting
		self.F = 0
		self.theta = self.state[2, c_step + 1]
		self.x = self.state[0, c_step + 1]
		self.y = self.state[1, c_step + 1]
		self.updateSensorPositions( self.x, self.y, self.theta )
		self.F = self.enviroment.getFood( self.x, self.y )