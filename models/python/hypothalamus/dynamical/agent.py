# --------------------------------------------------------------------
# Agent: Represents the physical body of the 
# --------------------------------------------------------------------
import numpy as np
from sensors import *
from brainsv2 import *
from body import *
import matplotlib.pyplot as plt

class AgentBuilder:
	@staticmethod
	def buildSimpleTemperatureAgent( environment, x, y ):
		# Sensors
		tempSensorL = TemperatureSensor( ego_angle = -np.pi/2.0, environment = environment )
		tempSensorR = TemperatureSensor( ego_angle = np.pi/2.0, environment = environment )
		# Brain
		brain = SimpleBrain()
		brain.registerSensor( 'Tl', tempSensorL )
		brain.registerSensor( 'Tr', tempSensorR )		

		# Agent
		a = Agent( environment, brain, x = x, y = y, theta = np.pi/2.0 )
		a.addSensor( tempSensorR )
		a.addSensor( tempSensorL )

		return a

	@staticmethod
	def buildMotivationalAgent( environment, x, y ):
		# Sensors
		tempSensorL = TemperatureSensor( ego_angle = -np.pi/2.0, environment = environment )
		tempSensorR = TemperatureSensor( ego_angle = np.pi/2.0, environment = environment )
		chemSensorL = ChemicalSensor( ego_angle = -np.pi/2.0, environment = environment )
		chemSensorR = ChemicalSensor( ego_angle = np.pi/2.0, environment = environment )

		# Brain
		brain = MotivationalBrain( Tb = 37.0, E = 1.0 )
		brain.registerSensor( 'Tl', tempSensorL )
		brain.registerSensor( 'Tr', tempSensorR )	
		brain.registerSensor( 'Fl', chemSensorL )
		brain.registerSensor( 'Fr', chemSensorR )

		# Agent
		a = Agent( environment, brain, x = x, y = y, theta = np.pi/2.0 )
		a.addSensor( tempSensorR )
		a.addSensor( tempSensorL )
		a.addSensor( chemSensorR )
		a.addSensor( chemSensorL )

		return a	

class Agent:

	def __init__(self, environment, brain, x, y, theta, radius = 2.0, record = [] ):
		
		self.body = Body(x, y, theta0 = theta)
		# Setting up brain and recorder
		self.recorder = Recorder( brain.state_labels )
		brain.setRecorder( self.recorder )
		brain.setBody( self.body )
		self.brain = brain
		self.environment = environment
		self.sensors = []

		self.updateSensorPositions( x, y, theta )

	def getTransformation( self, x, y, theta ):
		return np.array([[np.cos(theta), -np.sin(theta), x], 
						 [np.sin(theta), np.cos(theta), y], 
						 [0, 0, 1]])

	def updateSensorPositions( self, x, y, theta ):
		p = np.array([self.body.radius, 0, 1])
		M = self.getTransformation( x, y, theta )

		for s in self.sensors:
			R = self.getTransformation( 0, 0, s.ego_angle )
			new_pos = np.dot(M, np.dot(R, p))
			s.setPosition( new_pos[0].item(), new_pos[1].item() )

	def addSensor( self, sensor ):
		self.sensors.append( sensor )

	def reset( self, tf, h ):
		m = int(tf/h)
		self.brain.reset( m )
		self.body.reset()

	def step( self, c_step, h, t ):
		
		x,y = self.body.getPosition()
		self.recorder.recordPosition( x, y, c_step )

		F = self.environment.getFood( x, y )
		self.brain.step(c_step, h, t, F)

		x,y = self.body.getPosition()
		theta = self.body.getOrientation()
		
		self.updateSensorPositions( x, y, theta )

	def draw( self, c_step, graphics ):
		x, y = self.body.getPosition()	
		self.recorder.plotTrace( ax, c_step, graphics )
		graphics.circle( x, y, self.body, color = 'k' )			

		for s in self.sensors:
			graphics.circle( s.pos[0], s.pos[1], 1.0, color = [0.5,0.5,0.5] )
		
		
