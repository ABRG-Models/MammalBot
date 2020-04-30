import numpy as np
import matplotlib.pyplot as plt

from PyQt5.QtCore import QMutex

import copy

class Recorder:
	def __init__( self, brain = None ):
		self.data = {}
		self.state = {}
		
		if brain is None:
			self.state_labels = []
		else:
			self.state_labels = brain.state_labels
			
			for v in  brain.state_labels:
				self.state[v] = []

			self.variables = brain.getAvailableVariables()

		self.time = []		
		self.step = 0	
		self.trace = None	

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
		
		if name in self.variables:
			return self.data[name][0:self.step]
		else:
			return []

	def clone( self ):
		# No deep copies, only the steps guaranties consistency
		r = Recorder()
		r.data = self.data
		r.step = self.step
		r.state = self.state
		r.time = self.time
		r.trace = self.trace
		r.state_labels = self.state_labels
		r.variables = self.variables

		return r