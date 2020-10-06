#!/usr/bin/python

import rospy
import miro2 as miro
import os

from std_msgs.msg import Float32MultiArray, UInt32MultiArray, Int16MultiArray
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import TwistStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError

import math
import numpy as np
# import matplotlib.pyplot as plt
import time
import threading

from hyp_controller import *


# Utility enums
MAX_ITERATIONS = 10
LEFT, RIGHT = range(2)
tilt, lift, yaw, pitch = range(4)
droop, wag, left_eye, right_eye, left_ear, right_ear = range(6)

class MiroController:

	def __init__( self ):
		# Set robot name
		topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")
		rospy.init_node("sign_stimuli", anonymous=True)
		# Define ROS publishers
		self.pub_cmd_vel = rospy.Publisher(topic_root + "/control/cmd_vel", TwistStamped, queue_size=0)
		self.pub_cos = rospy.Publisher(topic_root + "/control/cosmetic_joints", Float32MultiArray, queue_size=0)
		self.pub_illum = rospy.Publisher(topic_root + "/control/illum", UInt32MultiArray, queue_size=0)
		self.pub_kin = rospy.Publisher(topic_root + "/control/kinematic_joints", JointState, queue_size=0)

		# Subscribers
		#rospy.Subscriber(topic_root + '/sensors/package', miro.msg.sensors_package, self.touchListener)
		self.sub_caml = rospy.Subscriber(topic_root + "/sensors/caml/compressed",
										 CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
		self.sub_camr = rospy.Subscriber(topic_root + "/sensors/camr/compressed",
										 CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)
		self.sub_mics = rospy.Subscriber(topic_root + "/sensors/mics", Int16MultiArray, self.callback_mics)

		# Initializing object for data publishing
		self.velocity = TwistStamped()
		self.cos_joints = Float32MultiArray()
		self.cos_joints.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.kin_joints = JointState()
		self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
		self.kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
		self.illum = UInt32MultiArray()
		self.illum.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

		self.image = [None, None]
		self.image_converter = CvBridge()
		self.controller = HypothalamusController( self )
		self.running = True
		self.wag_t = 0.0

		self.audio = None
 
	def move( self, forces ):
		msg_cmd_vel = TwistStamped()

		# print forces
		# desired wheel speed (m/sec)
		wheel_speed = [forces[0], forces[1]]
		v = 1.0
		(dr, dtheta) = miro.utils.wheel_speed2cmd_vel(wheel_speed)
		msg_cmd_vel.twist.linear.x = dr
		msg_cmd_vel.twist.angular.z = dtheta

		# publish message to topic
		self.pub_cmd_vel.publish(msg_cmd_vel)

	def moveHead( self, y ):
		# self.kin_joints.position[lift] = math.radians(self.LiftControl.get_value())
		# self.kin_joints.position[pitch] = math.radians(self.PitchControl.get_value())
		self.kin_joints.position[lift] = math.radians(y)
		self.pub_kin.publish( self.kin_joints )

	def shine( self, color, idxs = range(6), off = False ):
		
		for i in idxs:
			# if off:
			#     self.illum.data[i] = 0x00000000
			# else:
			self.illum.data[i] = color

		self.pub_illum.publish(self.illum)


	def tailWag( self ):
		MAX_TIME = 10
		t = self.wag_t
		A = 1.0
		w = 2*np.pi*0.2
		f = lambda t: A*np.cos(w*t)

		if t > MAX_TIME:
			self.cos_joints.data[wag] = 0.5
			r = False
			self.wag_t = 0
		else:
			self.cos_joints.data[wag] = f(t)
			r = True

		self.cos_joints.data[droop] = 0.0
		self.pub_cos.publish(self.cos_joints)
		self.wag_t += 0.2


	def callback_cam(self, ros_image, camera = LEFT):

		# silently (ish) handle corrupted JPEG frames
		try:
			# convert compressed ROS image to raw CV image
			self.image[camera] = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")

		except CvBridgeError as e:

			# swallow error, silently
			# print(e)
			pass

	def callback_caml(self, ros_image):
		self.callback_cam( ros_image, LEFT )

	def callback_camr(self, ros_image):
		self.callback_cam( ros_image, RIGHT )

	def callback_mics(self, data):
		self.audio = data.data

	# Main loop
	def run( self ):
		h = 0.01
		t = 0.0

		while not rospy.core.is_shutdown() and self.running:
			if self.image[0] is not None and self.image[1] is not None:
				self.controller.step(self.image, h, t, self.audio)
				self.image = [None, None]

			
			plt.pause(0.01)
			t += h
			
			if t > 100.0:
				self.running = False

		self.controller.plots()


if __name__ == "__main__":
	mc = MiroController()
	mc.run()