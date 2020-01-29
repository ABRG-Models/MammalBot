import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import time
import copy
import multiprocessing
import threading
import rospy

import cv2

import node

import miro2 as miro
import signals

# Borrowed from node_action
from std_msgs.msg import Float32MultiArray
from action.action_types import ActionInput


class Drive(object):
	"""Interface for the drives"""
	def __init__(self):
		super(Drive, self).__init__()
		self.G = 0.4
		self.drive = 0.0
		self.mu = 0.0
		self.dmap = 0.0

	def tick(self, state):
		pass

	def get_drive(self):
		return self.drive

	def get_mu(self):
		return self.mu

	def map(self):
		return self.dmap

class TemperatureDrive(Drive):
	def __init__(self):
		super(TemperatureDrive, self).__init__()
		# Temperature dynamics parameters
		self.Tr = 0.0
		self.Tl = 0.0
		self.k1 = 0.99
		self.k2 = 1.0
		self.Tp = 0.6
		radius = 2.0
		self.A = 2*np.pi*radius

	def inject_temperature(self, Tl, Tr):
		self.Tl = Tr
		self.Tr = Tl

	def tick(self, state):
		# Competition helper function
		sigma_t = 15.0
		G_t = 10.0
		x0_t = 0.5
		ft = lambda x : np.pi*x/15.0 if abs(x)<15.0 else np.pi*np.sign(x)
		gt = lambda x : G_t/(1 + np.exp(-sigma_t*(x-x0_t)))
		# Ambient temperature
		Ta = (Tl + Tr)/2.0

		# Temperature drives
		wH = 1/np.abs(40.0 - 20.0)
		Tc = state.Tb
		self.drive = (1 + wH*np.abs(state.Tb - self.Tp))**2 - 1
		self.mu = gt(self.drive)
		self.dmap = self.G - k1*(state.Tb - Ta)*self.A - self.k2*(1 - self.A)*(state.Tb - Tc)


class SocialDrive(Drive):
	def __init__(self):
		super(SocialDrive, self).__init__()

		# Initializing parameters of the drive
		self.F = 0.0
		self.alpha = 0.01
		self.Ep = 0.9

	def inject_energy(self, F):
		self.F = F

	def get_drive(self):
		return self.drive

	def tick(self, state):
		# Food helper functions
		sigma_e = 10.0
		G_e = 10.0
		x0_e = 1.0
		I = 10.0
		sigma2 = 0.05
		fe = lambda x : np.pi*x if abs(x)<1.0 else np.pi*np.sign(x)
		ge = lambda x : G_e/(1 + np.exp(-sigma_e*(x-x0_e)))
		he = lambda x : I*np.exp(-(x - self.Ep)**2/(2*sigma2**2))
		# food drives
		wE = 1.0
		self.drive = (1 + wE*(self.Ep - state.E))**2 - 1.0
		self.mu = ge(self.drive)
		self.dmap = -self.alpha*self.G + self.F

class PlayDrive(Drive):
	def __init__(self):
		super(PlayDrive, self).__init__()

		# Initializing parameters of the drive
		self.F = 0.0
		self.alpha = 0.01
		self.Ep = 0.9

	def inject_energy(self, F):
		self.F = F

	def get_drive(self):
		return self.drive

	def tick(self, state):
		# Food helper functions
		sigma_e = 10.0
		G_e = 10.0
		x0_e = 1.0
		I = 10.0
		sigma2 = 0.05
		fe = lambda x : np.pi*x if abs(x)<1.0 else np.pi*np.sign(x)
		ge = lambda x : G_e/(1 + np.exp(-sigma_e*(x-x0_e)))
		he = lambda x : I*np.exp(-(x - self.Ep)**2/(2*sigma2**2))
		# food drives
		wE = 1.0
		self.drive = (1 + wE*(self.Ep - state.Eplay))**2 - 1.0
		self.mu = ge(self.drive)
		self.dmap = -self.alpha*self.G + self.F


class SaliencyPotential:
	def __init__(self):
		self.rate = 10
		self.dRho = 0.0

	def tick(self, state, drives):
		sigma_c = 0.5
		gc = lambda x,x0: np.exp(-(x - x0)**2/(2*sigma_c**2))/np.sqrt(2*np.pi*sigma_c**2)
		# Competition parameters
		a = np.abs(drives['social'].get_drive())
		b = np.abs(drives['play'].get_drive())

		n1 = (np.abs(a) + np.abs(b))
		a = a/n1
		b = b/n1
		# Potential
		state.U = lambda rho: (1.0/4.0)*rho**2*(1 - rho)**2 + a*rho**2 + b*(1 - rho)**2
		dU = lambda rho: (1.0/2.0)*(rho*((1-rho)**2 + a) - (1-rho)*(rho**2 + b))
		self.dRho = -self.rate*dU( state.rho )

	def map(self):
		return self.dRho

class BehaviourPattern( object ):
	def __init__( self, sys ):
		self.pars = sys.pars
		self.state = sys.state

	def motivate( self, motivation, vigor ):
		pass

class HeatSeekingBehaviourPattern( BehaviourPattern ):
	def __init__( self, sys ):
		super(HeatSeekingBehaviourPattern, self).__init__( sys )

	def motivate( self, motivation, vigor ):
		pass

class SocializeBehaviourPattern( BehaviourPattern ):
	def __init__( self, sys ):
		super(SocializeBehaviourPattern, self).__init__( sys )
		
	def motivate( self, motivation, vigor ):
		self.pars.spatial.face_gain = motivation


class PlayBehaviourPattern( BehaviourPattern ):
	def __init__(self, sys):
		super(PlayBehaviourPattern, self).__init__( sys )

	def motivate( self, motivation, vigor ):
		self.pars.spatial.ball_gain = motivation

class Behaviour:
	def __init__(self, sys):
		self.behaviour_patterns = [
		SocializeBehaviourPattern( sys ), PlayBehaviourPattern( sys )]
							  		

	def tick(self, state):
		sigma_c = 0.5
		gc = lambda x,x0: np.exp(-(x - x0)**2/(2*sigma_c**2))/np.sqrt(2*np.pi*sigma_c**2)

		for i in range(len(self.behaviour_patterns)):
			m = gc(state.rho, i)
			self.behaviour_patterns[i].motivate(m, state.mu)

		# dTheta_temperature = ft((Tb - Tp)**3*(Tl - Tr)) + np.random.rand() - 0.5
		# dTheta_energy = fe((Fl - Fr)*(Ep - E)*300) + np.random.rand() - 0.5
		# dTheta = )*dTheta_temperature + gc(rho, 1)*dTheta_energy

class NodeMotivation(node.Node):

	def __init__(self, sys):

		node.Node.__init__(self, sys, "motivation")

		self.input = sys.input
		self.state = sys.state
		# Initializing drives
		# self.drives = {'temperature' : TemperatureDrive(), 
		# 			   'social' : SocialDrive() }
		self.drives = {'social' : SocialDrive(), 
					   'play': PlayDrive() }
		# Initializing saliency potential
		self.motivation = SaliencyPotential()
		# Initializing behavioral systems
		self.behaviour = Behaviour( sys )	

		# Define ROS publishers	
		name = sys.topic_base_name 
		self.pub_cmd_m = rospy.Publisher( name + "motivation", Float32MultiArray )
		self.data = Float32MultiArray()


	def inject_temperature( self ):
	 	msg = self.input.sensors_package
	 	cliff = np.array(msg.cliff.data)

	 	self.drives['temperature'].inject_temperature( cliff[0], cliff[1] )

	def inject_social_energy( self ):
		E = self.state.stroke
		
		if E > 0:
			print "Feeding social: ", E

		self.drives['social'].inject_energy(E)

	def inject_play_energy( self ):
		E = 0.0

		if self.state.detect_objects_for_spatial[0] is None:
			return

		p = len(self.state.detect_objects_for_spatial[0].balls) 

		if p > 0:
			print "Feeding play: ", p
			E = 0.1

		self.drives['play'].inject_energy( E )

	def publish_state( self ):
		self.data.data = [self.state.E, self.state.Eplay]
		self.pub_cmd_m.publish(self.data)


	def tick( self ):
		# Evolve the drive dynamical system
		for d in self.drives:
			self.drives[d].tick( self.state )

		# Evolve the motivational dynamical system
		self.motivation.tick( self.state, self.drives )
		# Modify state
		self.state.mu = (self.drives['social'].get_mu() + self.drives['play'].get_mu())
		# self.state.Tb += self.state.dt*drives['temperature'].map()
		self.state.E += self.state.dt*self.drives['social'].map()
		self.state.Eplay += self.state.dt*self.drives['play'].map()
		self.state.rho += self.state.dt*self.motivation.map()
		# Spread towards motor and sensory systems
		self.behaviour.tick( self.state )

		#self.inject_temperature()
		self.inject_social_energy()
		self.inject_play_energy()

		self.publish_state()
