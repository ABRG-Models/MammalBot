#!/usr/bin/python

import rospy
import miro2 as miro
import os

from std_msgs.msg import Float32MultiArray, UInt32MultiArray
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import TwistStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError

import math
import numpy as np
import matplotlib.pyplot as plt 
import time
import threading

from hyp_controller import *
from test_controller import *


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
        rospy.Subscriber(topic_root + '/sensors/package', miro.msg.sensors_package, self.callback_package)
        self.sub_caml = rospy.Subscriber(topic_root + "/sensors/caml/compressed",
                                         CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
        self.sub_camr = rospy.Subscriber(topic_root + "/sensors/camr/compressed",
                                         CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)

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
        self.controller = GreenTestController( self )
        self.running = True
        self.obstacle = False
        self.v = [0,0]
 
    def reset_body( self ):
        self.cos_joints.data[wag] = 0.5
        self.cos_joints.data[droop] = 0.0
        self.pub_cos.publish(self.cos_joints)

        self.move_head( 0, 40 )
        time.sleep(1)

    def move( self, forces ):
        msg_cmd_vel = TwistStamped()
        h = 0.01
        k = 0.1
        m = 1.0
        v1 = self.v[0] + h*forces[0]/m - k*self.v[0]
        v2 = self.v[1] + h*forces[1]/m - k*self.v[1]
        # print forces
        # desired wheel speed (m/sec)
        wheel_speed = [v1, v2]
        v = 1.0
        (dr, dtheta) = miro.utils.wheel_speed2cmd_vel(wheel_speed)
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # publish message to topic
        self.pub_cmd_vel.publish(msg_cmd_vel)
        self.v = [v1, v2]

    def stop( self ):
        msg_cmd_vel = TwistStamped()
        self.v = [0.0, 0.0]
        wheel_speed = self.v
        (dr, dtheta) = miro.utils.wheel_speed2cmd_vel(wheel_speed)
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # publish message to topic
        self.pub_cmd_vel.publish(msg_cmd_vel)
    
    def move_head( self, x, y = 40 ):
        # self.kin_joints.position[lift] = math.radians(self.LiftControl.get_value())
        self.kin_joints.position[yaw] = math.radians(x)
        self.kin_joints.position[lift] = math.radians(y)
        self.pub_kin.publish( self.kin_joints )

    def shine( self, color, idxs = range(6), off = False ):
		
        for i in idxs:
            # if off:
            #     self.illum.data[i] = 0x00000000			
            # else:
            self.illum.data[i] = color			

        self.pub_illum.publish(self.illum)
        

    def tail_wag( self, w ):        
        self.cos_joints.data[wag] = w
        self.cos_joints.data[droop] = 0.0
        self.pub_cos.publish(self.cos_joints)

    # ---- CALLBACKS ----

    def callback_package(self, msg):

		# store for processing in update_gui
        self.obstacle = msg.sonar.range < 0.1

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

    # Main loop
    def run( self ):
        h = 0.01
        t = 0.0

        self.reset_body()

        while not rospy.core.is_shutdown() and self.running:
            if self.image[0] is not None and self.image[1] is not None:                
                self.controller.step(self.image, h, t)                
                self.image = [None, None]           

            if self.obstacle:
                self.controller.cancel()
                self.obstacle = False

            self.controller.plots()
            plt.pause(0.01)
            t += h
            
        

if __name__ == "__main__":
    mc = MiroController()
    mc.run()