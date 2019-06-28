# This is a newly created BRAHMS Process. It is a non-native
# process, and does not need to be built - BRAHMS can run it
# as it stands.


import brahms
import numpy

import sys
# Needed for importing rospy in embedded scenario
if not hasattr(sys, 'argv'):
    sys.argv  = ['']

import os
import rospy
import miro2 as miro
import numpy as np
from geometry_msgs.msg import TwistStamped


# Set robot name
topic_root = "/" + os.getenv("MIRO_ROBOT_NAME")

# event function
def brahms_process(persist, input):

	# nominal output
	output = {'event':{'response':S_NULL}}


	# switch on event type
	if input['event']['type'] == EVENT_MODULE_INIT:

		# provide component information
		output['info'] = {}
		output['info']['component'] = (0, 1)
		output['info']['additional'] = ('Author=David Buxton\n')
		output['info']['flags'] = (F_NOT_RATE_CHANGER)
		
		# ok
		output['event']['response'] = C_OK
	

	# switch on event type
	elif input['event']['type'] == EVENT_STATE_SET:

		# ok
		output['event']['response'] = C_OK


	# switch on event type
	elif input['event']['type'] == EVENT_INIT_CONNECT:

		# on first call
		if input['event']['flags'] & F_FIRST_CALL:

			# Access input port 'python_ros_in'
			index = input['iif']['default']['index']['python_ros_in']
			port = input['iif']['default']['ports'][index]
			
			# Initialise ROS nodes
			persist['ros_pub'] = rospy.Publisher(topic_root + "/control/cmd_vel", TwistStamped, queue_size=10)
			rospy.init_node('output', anonymous=True)

			# Store data
			persist['python_ros_in'] = port['data']
			persist['velocity'] = TwistStamped()

			# do nothing
			pass

		# on last call
		if input['event']['flags'] & F_LAST_CALL:

			# do nothing
			pass

		# ok
		output['event']['response'] = C_OK


	# switch on event type
	elif input['event']['type'] == EVENT_RUN_SERVICE:

		# Set wheel speeds as input data
		wheel_speed = [persist['python_ros_in'], persist['python_ros_in']]

		# Convert to command velocity
		(dr, dtheta) = miro.utils.wheel_speed2cmd_vel(wheel_speed)

		# Set velocity values
		persist['velocity.twist.linear.x'] = dr
		persist['velocity.twist.angular.z'] = dtheta

		# Publish data to ROS node
		persist['ros_pub'].publish(persist['velocity'])

		# ok
		output['event']['response'] = C_OK


	# return
	return (persist, output)
