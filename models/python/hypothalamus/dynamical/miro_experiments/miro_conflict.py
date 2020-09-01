#!/usr/bin/python

import rospy
import miro2 as miro
import os

from std_msgs.msg import Float32MultiArray, UInt32MultiArray
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import TwistStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import matplotlib.pyplot as plt 
import time
import threading


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

        # Subscribers
        #rospy.Subscriber(topic_root + '/sensors/package', miro.msg.sensors_package, self.touchListener)
        self.sub_caml = rospy.Subscriber(topic_root + "/sensors/caml/compressed",
                                         CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
        self.sub_camr = rospy.Subscriber(topic_root + "/sensors/camr/compressed",
                                         CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)

        # Initializing object for data publishing
        self.velocity = TwistStamped()
        self.cos_joints = Float32MultiArray()
        self.cos_joints.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.image = [None, None]
        self.image_converter = CvBridge()
        self.m = 240
        self.n = 320
        self.percept = [np.zeros((self.m, self.n)), np.zeros((self.m, self.n))]
        self.running = True


    def process(self, im, camera = LEFT):
        if im is None:
            return

        frame = cv2.resize(im, (self.n, self.m))
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray,(21,21), 0)
        ret, thres = cv2.threshold( gray, 0, 255, cv2.THRESH_OTSU )
        
        
        self.percept[camera] = blur if np.random.rand() > 0.97  else self.percept[camera] - blur
        

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

    def evolve_perception( self ):
        while True:
            self.wp.evolve( self.media )
            time.sleep( 0.0001 )

    # Main loop
    def run( self ):
        
        fig, ax = plt.subplots(1, 2)
        while not rospy.core.is_shutdown() and self.running:
            for i in range(2):
                if self.image[i] is not None:                
                    self.process(self.image[i], i)
                    self.image[i] = None
            
            stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
            try:
                disparity = stereo.compute(self.percept[0], self.percept[1])
            except:
                disparity = self.percept[0]

            ax[0].imshow( self.percept[0] )
            ax[1].imshow( disparity )
            plt.pause(0.01)
            
        

if __name__ == "__main__":
    mc = MiroController()
    mc.run()