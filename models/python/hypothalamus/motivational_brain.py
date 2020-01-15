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

class MiroController:

	def __init__( self ):
		self.actions = [ self.earWiggle, self.tailWag, self.rotate, self.nod ]

		# Set robot name
		topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")
		# Python needs to initialise a ROS node for publishing data
		rospy.init_node("kbandit", anonymous=True)
		# Define ROS publishers
		self.pub_cmd_vel = rospy.Publisher(topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0)
		self.pub_cos = rospy.Publisher(topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0)
		self.pub_illum = rospy.Publisher(topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0)
		self.pub_kin = rospy.Publisher(topic_base_name + "/control/kinematic_joints", JointState, queue_size=0)

		# Initializing object for data publishing
		self.velocity = TwistStamped()
		self.kin_joints = JointState()
		self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
		self.kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
		self.cos_joints = Float32MultiArray()
		self.cos_joints.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.illum = UInt32MultiArray()
		self.illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

	# Main loop
	def run( self ):
		running = True
		r = np.random.randint( 0, len(self.actions) )
		t = 0.0
		h = 0.1

		while( running ):
			if !self.actions[r]( t ):
				# Action selection
				r = np.random.randint( 0, len(self.actions) )
				t = 0.0
			
			t = t + h			
			time.sleep(0.01)

if __name__ == "__main__":
	mc = MiroController()
	mc.run()