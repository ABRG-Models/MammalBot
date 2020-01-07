#!/usr/bin/python
#
# Miro controller
# 

class controller:

	def callback_package(self, msg):

		# ignore until active
		if not self.active:
			return

		# store
		self.package = msg

	def step(self, bg, ctx, lh):

		# output
		msg_cmd_vel = TwistStamped()
		# desired wheel speed (m/sec)
		wheel_speed = [0.0, 0.0]

		if rospy.core.is_shutdown():
			return -1

		# check if both are available
		if ctx.hasImage():

			w = ball[0]
			rad = ball[1]

			l = w[0]
			r = w[2]

			print l, r, rad

			# convert to wheel speed (m/sec)
			k = 0.4
			d = r - l
			target_rad = 60
			drad = rad - target_rad
			krad = 0.002
			wheel_speed[0] = k * d + krad * drad
			wheel_speed[1] = k * -d + krad * drad
			
			# filter wheel speed
			gamma = 0.25
			
			for i in range(2):
				self.wheel_speed[i] += gamma * (wheel_speed[i] - self.wheel_speed[i])

			# convert wheel speed to command velocity (m/sec, Rad/sec)
			(dr, dtheta) = miro.utils.wheel_speed2cmd_vel(self.wheel_speed)

			# update message to publish to control/cmd_vel
			msg_cmd_vel.twist.linear.x = dr
			msg_cmd_vel.twist.angular.z = dtheta

			# publish message to topic
			self.pub_cmd_vel.publish(msg_cmd_vel)

			return 0
		

	def __init__(self, args):

		#Create object to convert ROS images to OpenCV format
		self.image_converter = CvBridge()	
		self.init_ brain()

		rospy.init_node("client_shepherd", anonymous=True)

		# state
		self.t_now = 0.0
		self.im = [None, None]
		self.wheel_speed = [0.0, 0.0]

		# inputs
		self.active = False
		self.package = None

		# handle args
		for arg in args:
			f = arg.find('=')
			if f == -1:
				key = arg
				val = ""
			else:
				key = arg[:f]
				val = arg[f+1:]
			if key == "pass":
				pass
			else:
				error("argument not recognised \"" + arg + "\"")

		
		# wait for connect
		print "wait for connect..."
		time.sleep(1)