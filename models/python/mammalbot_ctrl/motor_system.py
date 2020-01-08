class MotorSystem:
	def __init__(self, miro_client):
		self.miro_client = miro_client

	def step( self, model ):
		# Computing output

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

            self.miro_client.pub_cmd_vel_ms( self.wheel_speed[0], self.wheel_speed[1] )

            return 0