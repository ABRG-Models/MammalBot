#!/usr/bin/python
#
# Miro cortex - object detection/classification

class Cortex:
	def __init__(self):
		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# publish
		topic = topic_base_name + "/control/cmd_vel"
		print ("publish", topic)
		self.pub_cmd_vel = rospy.Publisher(topic, TwistStamped, queue_size=0)

		# subscribe
		topic = topic_base_name + "/sensors/package"
		print ("subscribe", topic)
		self.sub_package = rospy.Subscriber(topic, miro.msg.sensors_package, self.callback_package, queue_size=1, tcp_nodelay=True)

		self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed",
					CompressedImage, self.callback_caml)

		self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed",
					CompressedImage, self.callback_camr)


	def callback_cam(self, ros_image, index):

		# silently (ish) handle corrupted JPEG frames
		try:

			# convert compressed ROS image to raw CV image
			image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")

			# do zoom
			#image = cv2.resize(image, (int(image.shape[1] * 0.5), int(image.shape[0] * 0.5)))

			# store image
			self.im[index] = image

		except CvBridgeError as e:

			# swallow error, silently
			#print(e)
			pass

	def callback_caml(self, ros_image):

		self.callback_cam(ros_image, 0)

	def callback_camr(self, ros_image):

		self.callback_cam(ros_image, 1)

	def hasImage( self ):
		return (not self.im[0] is None) and (not self.im[1] is None)

	def step( self ):
		# Return the appropriate to the BG
		# copy frames
		im = self.im
		# clear both frames
		self.im = [None, None]
		# send to image processor
		ball = processImages(im)