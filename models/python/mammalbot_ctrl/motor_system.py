class MotorSystem:
	def __init__(self):
		topic = topic_base_name + "/control/cmd_vel"
        print ("publish", topic)
        self.pub_cmd_vel = rospy.Publisher(topic, TwistStamped, queue_size=0)
        self.wheel_speed = [0.0, 0.0]
        self.package = None

	def step( self, model ):
		# Computing output
		
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