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
            # Create output port with label 'bg_input'
            # The number here must match the number of BG channels
            persist['bg_input'] = brahms.operation(
                persist['self'],
                OPERATION_ADD_PORT,
                '',
                'std/2009/data/numeric',
                'DOUBLE/REAL/6',
                'bg_input'
            )

            # Initialise ROS nodes
            persist['ros_bg'] = rospy.Publisher("/basal_ganglia/mctx", String, queue_size=10)
            rospy.init_node('spineml_bg', anonymous=True)

            # Access MCtx output
            index = input['iif']['default']['index']['mctx_out']
            port = input['iif']['default']['ports'][index]

            # Store data
            persist['mctx_data'] = port['data']

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
        print(persist['mctx_data'])

        # TODO: Run motor program associated with output channels of activity level 0.95 or above
        # TODO: Create standalone Python motor programs for each action

        # Set input values
        brahms.operation(
            persist['self'],
            OPERATION_SET_CONTENT,
            persist['bg_input'],
            np.array([0.7, 0.2, 0.5, 0.1, 0, 0], np.float64)
        )

        # Send output values to ROS
        persist['ros_bg'].publish(str(persist['mctx_data']))

        # OK
        output['event']['response'] = C_OK

    # Return
    return persist, output
