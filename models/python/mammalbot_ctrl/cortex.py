#!/usr/bin/python
#
# Miro cortex - object detection/classification

class Cortex:
	def __init__(self):

		self.image_converter = CvBridge()	
		self.im = [None, None]
		

	# def callback_cam(self, ros_image, index):

	# 	# silently (ish) handle corrupted JPEG frames
	# 	try:
	# 		# convert compressed ROS image to raw CV image
	# 		image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")

	# 		# do zoom
	# 		#image = cv2.resize(image, (int(image.shape[1] * 0.5), int(image.shape[0] * 0.5)))

	# 		# store image
	# 		self.im[index] = image

	# 	except CvBridgeError as e:

	# 		# swallow error, silently
	# 		#print(e)
	# 		pass

	def hasImage( self ):
		return (not self.im[0] is None) and (not self.im[1] is None)

	def step( self, inputs ):
		# Return the appropriate to the BG
		# copy frames
		im = miroClient.pop_images()
		# send to image processor
		ball = detector.processImages(im)

		return 


