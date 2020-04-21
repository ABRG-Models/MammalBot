import numpy as np
import matplotlib.pyplot as plt

class Recorder:
	def __init__( self, state_labels ):
		self.data = {}
		self.state_labels = state_labels
		

	def reset( self, m, state, variables ):
		self.trace = np.zeros( (2, m) )		
		self.state = np.zeros( (len(self.state_labels), m) )

		for v in variables:
			self.data[v] = np.zeros(m)

	def recordState( self, state, step ):
		self.state[:,step] = state

	def recordPosition( self, x, y, step ):
		self.trace[:,step] = [x,y]

	def recordVariable( self, name, value, step ):
		if name not in self.data:
			raise Exception('Recorder exception', 'The variable has not been set up to be recorded')

		self.data[name][step] = value 

	def plotStateVariables( self, graphics, time, step):
		if len(self.state_labels) == 0:
			return 
			
		if name not in self.state_labels:
			raise Exception('Recorder error', 'Name not a state variable')

		idx = self.state_labels.index(name)
		#plt.sca(ax)	
		#plt.cla()

		graphics.plot( xdata, ydata, name )
		

	def registerVariable( self, name, pos, alias = None, axis = None  ):


	def plotTrace( self, graphics, step ):
		graphics.trace( self.trace[0,:step], self.trace[1,:step] )
