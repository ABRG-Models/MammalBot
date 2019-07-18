# System imports
import os
import sys
# Needed for importing rospy in embedded scenario
if not hasattr(sys, 'argv'):
	sys.argv = ['']

# Required imports
import brahms
import rospy
import miro2 as miro

# ROS message types
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, UInt16MultiArray, UInt8MultiArray, UInt16, Int16MultiArray, String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, BatteryState, Imu, Range, CompressedImage

# Extra imports
import numpy as np


# Event function
def brahms_process(persist, input):

	# Nominal output
	output = {'event': {'response': S_NULL}}

	# Switch on event type
	if input['event']['type'] == EVENT_MODULE_INIT:

		# Component information
		output['info'] = {}
		output['info']['component'] = (0, 1)
		output['info']['additional'] = ('Author=David Buxton\n')
		output['info']['flags'] = (F_NOT_RATE_CHANGER)
		
		# OK
		output['event']['response'] = C_OK
		
	# Switch on event type
	elif input['event']['type'] == EVENT_STATE_SET:

		# OK
		output['event']['response'] = C_OK

	# Switch on event type
	elif input['event']['type'] == EVENT_INIT_CONNECT:

		# On first call
		if input['event']['flags'] & F_FIRST_CALL:

			# Initialise ROS nodes
			persist['ros_test'] = rospy.Publisher("/basal_ganglia/ctx", String, queue_size=10)
			rospy.init_node('spineml_bg_in', anonymous=True)

			# Access Ctx output
			index = input['iif']['default']['index']['ctx_out']
			port = input['iif']['default']['ports'][index]

			# Store data
			persist['ctx_data'] = port['data']

			# Do nothing
			pass

		# On last call
		if input['event']['flags'] & F_LAST_CALL:

			# Do nothing
			pass

		# OK
		output['event']['response'] = C_OK

	# Switch on event type
	elif input['event']['type'] == EVENT_RUN_SERVICE:

		# DEBUG data
		print(persist['ctx_data'])

		# TODO: Create standalone Python action request programs for each action
		# TODO: Pass input salience of each action request to BG model

		# OK
		output['event']['response'] = C_OK

	# Return
	return persist, output
