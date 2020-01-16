#!/usr/bin/python

import rospy
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage

import numpy as np
import time
import miro2 as miro

#Generate a fake enum for joint arrays
tilt, lift, yaw, pitch = range(4)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)
freq, volume, duration = range(3)
front_left, mid_left, rear_left, front_right, mid_right, rear_right = range(6)

def temperature_map( u, Tp, Tl, Tr, G, k1, k2, A ):

	# Nonlinearities
	sigma = 5.0
	vmax = 5.0
	x0 = 1.0
	f = lambda x : np.pi*x/15.0 if abs(x)<15.0 else np.pi*np.sign(x)
	g = lambda x : vmax/(1 + np.exp(-sigma*(x-x0)))
	# Ambient temperature
	Ta = (Tl + Tr)/2.0
	# State variables
	theta = u[2]
	Tb = u[3]
	E = u[4]
	# Drives
	wH = 1/np.abs(40.0 - 30.0)
	dHeat = (1 + wH*np.abs(Tb - Tp))**2 - 1
	# Motivation
	mu = g(dHeat)
	
	# Parameters
	k2 = 1.0
	Tc = Tb # No contact
	alpha = 0.5

	# Diff Equations	
	dx = mu*vmax*np.array([np.cos(theta), np.sin(theta)])
	dTb = G - k1*(Tb - Ta)*A - k2*(1 - A)*(Tb - Tc)
	dE = -alpha*G
	dTheta = f((Tb - Tp)**3*(Tl - Tr)) + np.random.rand() - 0.5

	return np.array([dx[0], dx[1], dTheta, dTb, dE ])

class MiroController:

	def __init__( self ):
		self.actions = [ self.earWiggle, self.tailWag, self.rotate, self.nod ]

		# Set robot name
		topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")
		# Python needs to initialise a ROS node for publishing data
		rospy.init_node("motivational", anonymous=True)
		# Define ROS publishers
		self.pub_cmd_vel = rospy.Publisher(topic_root + "/control/cmd_vel", TwistStamped, queue_size=0)

		# Subscribers
		rospy.Subscriber( topic_root + "/sensors/package", miro.msg.sensors_package, self.callback_package )

		# Initializing object for data publishing
		self.velocity = TwistStamped()

	def callback_package( self, msg ):
		self.cliff = msg.cliff.data

	def init_state( self ):
		radius = 1.0
		x = 0.0
		y = 0.0
		Tb = 0.0
		Tp = 0.5
		radius = 2.0
		k1 = 0.99
		G = 0.4

		self.state = np.array([x, y, theta, Tb, 1.0, 0.0]); # x, y, theta, Tb, E, rho
		self.Tp = Tp # Prefered temperature
		self.G = G # Heat generation rate
		self.k1 = k1
		self.A = 2*np.pi*radius
		self.radius = radius
		self.h = 0.01

	# Publish wheel speeds (radians)
    def pub_cmd_vel_rad(self, dr, dtheta):
        self.velocity.twist.linear.x = dr
        self.velocity.twist.angular.z = dtheta
        self.pub_cmd_vel.publish(self.cmd_vel_msg)	

	def step( self ):
		# Get sensor lectures
		if self.cliff is None:
			return

		Tl = self.cliff[0]
		Tr = self.cliff[1]
		# Step map
		ds = temperature_map( u, self.Tp, Tl, Tr, self.G, self.k1, 0.0, self.A )
		self.state = self.state + self.h*ds
		# Publish wheel speed
		x = ds[0]
		y = ds[1]
		dr = np.sqrt( x**2 + y**2 )
		theta = np.arctan( y/x )
		self.pub_cmd_vel_rad( dr, theta )


	# Main loop
	def run( self ):
		running = True
		
		self.init_state()

		while( running ):			
			self.step()
			time.sleep(0.01)

if __name__ == "__main__":
	mc = MiroController()
	mc.run()