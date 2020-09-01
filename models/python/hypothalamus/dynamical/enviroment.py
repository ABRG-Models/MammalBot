import numpy as np
import matplotlib.pyplot as plt
# --------------------------------------------------------------------
# Enviroment: Manages enviroment drawing and signals
# --------------------------------------------------------------------
class Environment:
	def __init__( self, xmin, xmax ):
		self.food_sources = []
		sigma = 100.0
		self.g = lambda x,y,x0,y0: np.exp(-(( x - x0)**2 + (y - y0)**2)/(2*sigma**2) )
		self.Tmin = 15.0
		self.Tmax = 45.0
		self.T = lambda x,y : ((self.Tmax - self.Tmin)/(xmax - xmin))*(x - xmin) + self.Tmin
		self.time = None
		self.ax_gradient = None

	def addFoodSource( self, x, y ):
		self.food_sources.append((x, y))

	def getTemperature( self, x, y ):
		return self.T(x,y)

	def getFoodSignal( self, x, y ):		
		signal = 0

		for i in range(len(self.food_sources)):
			x0,y0 = self.food_sources[i]
			signal = max( signal, self.g( x,y,x0,y0 ))

		return signal

	def getFood( self, x, y ):		
		# print('Getting food source!') 
		for i in range(len(self.food_sources)):
			x0,y0 = self.food_sources[i]
			# print('Position of the food source: {},{}'.format(x0,y0))

			if( np.linalg.norm(np.array([x-x0, y-y0])) < 10.0 ):
				print( 'Got food source' )
				return 0.3
			
		return 0.0


