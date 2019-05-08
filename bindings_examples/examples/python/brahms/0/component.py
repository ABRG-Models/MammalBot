
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

		# EXAMPLE
		#
		#print persist['state']['parameter_1']
		#print persist['state']['parameter_2']

		# ok
		output['event']['response'] = C_OK



	# switch on event type
	elif input['event']['type'] == EVENT_INIT_CONNECT:

		# on first call
		if input['event']['flags'] & F_FIRST_CALL:

			# EXAMPLE: access input port "X"

			index = input['iif']['default']['index']['python_in']
			port = input['iif']['default']['ports'][index]
			
			# EXAMPLE: check it is valid (2 by 4 matrix)
			#
			#if not port['structure'] == 'DOUBLE/REAL/2,4':
			#	output['error'] = "input 'X' is invalid (" + port['structure'] + ")";
			#	return(persist, output)

			# EXAMPLE: store handle to its data object
			
			persist['python_in'] = port['data']

			# create output
			persist['hOutputPort'] = brahms.operation(persist['self'], OPERATION_ADD_PORT, '', 'std/2009/data/numeric', 'DOUBLE/REAL/1', 'python_out')

			# do nothing
			pass

		# on last call
		if input['event']['flags'] & F_LAST_CALL:

			# EXAMPLE: create scalar output "Y" (and store its handle in hOutputPort)
			#
			#persist['hOutputPort'] = brahms.operation(
			#	persist['self'],
			#	OPERATION_ADD_PORT,
			#	'',
			#	'std/2009/data/numeric',
			#	'DOUBLE/REAL/1',
			#	'Y'
			#)

			# do nothing
			pass

		# ok
		output['event']['response'] = C_OK



	# switch on event type
	elif input['event']['type'] == EVENT_RUN_SERVICE:

		# EXAMPLE: get data from input
		
		input_value = persist['python_in']

		# do computation...

		# EXAMPLE: set value of output
		
		brahms.operation(
			persist['self'],
			OPERATION_SET_CONTENT,
			persist['hOutputPort'],
			numpy.array([input_value * 2], numpy.float64)
		)

		#print(persist['python_in'])

		# ok
		output['event']['response'] = C_OK



	# return
	return (persist, output)
