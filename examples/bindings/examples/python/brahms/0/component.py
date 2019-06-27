# This is a newly created BRAHMS Process. It is a non-native
# process, and does not need to be built - BRAHMS can run it
# as it stands.


import brahms
import numpy

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

			# Access input port 'python_in'
			index = input['iif']['default']['index']['python_in']
			port = input['iif']['default']['ports'][index]
			
			# Store data			
			persist['python_in'] = port['data']

			# Create output port with label 'python_out'
			persist['hOutputPort'] = brahms.operation(persist['self'], OPERATION_ADD_PORT, '', 'std/2009/data/numeric', 'DOUBLE/REAL/1', 'python_out')

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
		input_value = persist['python_in']

		# Set output value to input * 2
		brahms.operation(
			persist['self'],
			OPERATION_SET_CONTENT,
			persist['hOutputPort'],
			numpy.array([input_value * 2], numpy.float64)
		)

		# ok
		output['event']['response'] = C_OK


	# return
	return (persist, output)
