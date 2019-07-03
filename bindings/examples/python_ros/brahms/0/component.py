# This is a newly created BRAHMS Process. It is a non-native
# process, and does not need to be built - BRAHMS can run it
# as it stands.


import brahms
import numpy

import sys
# Needed for importing rospy in embedded scenario
if not hasattr(sys, 'argv'):
    sys.argv  = ['']

import rospy
from std_msgs.msg import Float64


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
			persist['ros_pub'] = rospy.Publisher("/example/python", Float64, queue_size=0)
			rospy.init_node('output', anonymous=True)

			# Store data
			persist['python_ros_in'] = port['data']

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

		# Move input data to local variable	
		ros_data = persist['python_ros_in']

		# Publish data to ROS node
		persist['ros_pub'].publish(ros_data)

		# ok
		output['event']['response'] = C_OK


	# return
	return (persist, output)
