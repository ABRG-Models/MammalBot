import numpy as np 

# Maps
from brainsv2 import *
from enviroment import *
from sensors import *
from recorder import *
from graphics import *
from agent import *
import time

from PyQt5.QtCore import pyqtSignal, QRunnable, QObject, QMutex


# --------------------------------------------------------------------
# Simlation: Manages the simulation
# --------------------------------------------------------------------
class Simulation( QRunnable ):
	def __init__(self, xmin, xmax, maxT = 10.0 ):
		super( Simulation, self ).__init__()
		
		self.h = 0.01
		self.maxT = maxT
		self.model = Model( xmin, xmax )
		self.signals = SimulationSignals()
		self.is_stop = False
		self.is_pause = False

	def reset( self, tf, h ):
		self.model.reset( tf, h )

	def stop( self ):
		self.is_stop = True

	def pause( self ):
		self.is_pause = not(self.is_pause)
		self.signals.pause.emit()

	def resume( self ):
		self.is_pause = False

	def run( self ):		
		self.t = 0.0
		mutex = QMutex()

		self.signals.start.emit()

		self.reset( self.maxT, self.h )

		while self.t < self.maxT:
			if self.is_stop:
				break

			if self.is_pause:
				continue

			mutex.lock()
			self.model.step( self.h, self.t )
			mutex.unlock()
			
			# pause time
			self.t += self.h
			# Emit drawing signal
			self.signals.draw.emit( self.model )			

			time.sleep( 0.01 )
			


		self.signals.finish.emit()


class Model:
	def __init__( self, xmin=0, xmax=1 ):
		self.environment = Environment( xmin, xmax )
		self.agents = []
		self.observed = None

	def reset( self, tf, h ):		
		for a in self.agents:
			a.reset( tf, h )

	def step( self, h, t ):
		for a in self.agents:
			a.step( h, t )

	def addAgent( self, a, observe = 1.0 ):
		a.envirnoment = self.environment
		self.agents.append( a )
		self.observed = len(self.agents)-1

	def getObservedAgent( self ):
		if len( self.agents ) > 0 and self.observed is not None:
			return self.agents[ self.observed ]
		else:
			return None

	def addFoodSource( self, x, y ):
		self.environment.addFoodSource( x, y )

	def clone( self ):
		m = Model()
		m.environment = self.environment
		m.agents = [a.clone() for a in self.agents ]
		m.observed = self.observed

		return m


class SimulationSignals( QObject  ):
	draw = pyqtSignal( object )
	finish = pyqtSignal()
	start = pyqtSignal()
	pause = pyqtSignal()
