import numpy as np
import matplotlib.pyplot as plt

class Recorder:
	def __init__( self, brain = None ):
		self.data = {}
		self.state = {}
		
		if brain is None:
			self.state_labels = []
		else:
			self.state_labels = brain.state_labels

		self.time = []

		for v in  brain.state_labels:
			self.state[v] = []

		self.variables = brain.getAvailableVariables()
		self.step = 0		

	def reset( self, m ):
		self.trace = np.zeros( (2, m) )	
		self.time = np.zeros(m)

		for v in self.state_labels:
			self.state[v] = np.zeros( m )

		self.step = 0

		print("Initializing recorder: ", m)

		for v in self.variables:
			self.data[v] = np.zeros(m)

	def recordSnapshot( self, t, x, y, brain ):
		for i in range(len(brain.state_labels)):
			self.state[self.state_labels[i]][self.step] = brain.state[i]

		self.trace[:,self.step] = [x,y]
		self.time[self.step] = t

		for name in self.variables:
			self.data[name][self.step] = brain.queryVariable( name )

		self.step += 1

	def getStateData( self, name ):
		return self.state[name][0:self.step]

	def getTrace( self ):
		return self.trace[:,0:self.step]

	def getTime( self ):
		return self.time[0:self.step]

	def getVariableData( self, name ):
		
		if name in self.variables.keys():
			return self.data[name]
		else:
			return []