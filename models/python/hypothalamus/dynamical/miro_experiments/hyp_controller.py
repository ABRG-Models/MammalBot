import sys
sys.path.append('..')
import brainsv2 as brains
from sensors import *
import cv2
import numpy as np
# import matplotlib.pyplot as plt
import entropy_percept as ep

from dynamical.audio_perception import AudioPerception

class VisualPerception:

	def isClose( self, images, r ):

		m1 = np.mean(images[0])
		m2 = np.mean(images[1])
		# print "m1: ", m1, ", m2: ", m2, ", size: ", r

		if r > 60:
			return True

		return False

	def getColor( self, image, x, y, r ):
		if image is None:
			return None

		image[0] = cv2.equalizeHist(image[0])
		image[1] = cv2.equalizeHist(image[1])
		image[2] = cv2.equalizeHist(image[2])
		boundaries = [
		([17, 100, 15], [50, 230, 56]),
		([86, 31, 4], [220, 88, 50])
		]

		# for (lower, upper) in boundaries:
		lower, upper = boundaries[0]
		# create NumPy arrays from the boundaries
		lower = np.array(lower, dtype = "uint8")
		upper = np.array(upper, dtype = "uint8")
		# find the colors within the specified boundaries and apply
		# the mask
		mask = cv2.inRange(image, lower, upper)
		output_g = cv2.bitwise_and(image, image, mask = mask)

		lower, upper = boundaries[1]
		# create NumPy arrays from the boundaries
		lower = np.array(lower, dtype = "uint8")
		upper = np.array(upper, dtype = "uint8")
		# find the colors within the specified boundaries and apply
		# the mask
		mask = cv2.inRange(image, lower, upper)
		output_r = cv2.bitwise_and(image, image, mask = mask)

		mg = np.mean( output_g )
		mr = np.mean( output_r )

		if mr > mg:
			return 'red'
		else:
			return 'green'

	def _locateCircle( self, img ):
		# img = cv2.resize(img, (320, 240))
		gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
		# Blur using 3 * 3 kernel.
		gray_blurred = cv2.blur(gray, (3, 3))

		# Apply Hough transform on the blurred image.
		detected_circles = cv2.HoughCircles(gray_blurred,
						cv2.HOUGH_GRADIENT, 1, 2, param1 = 90,
					param2 = 50, minRadius = 2, maxRadius = 200)

		return detected_circles

	def _filterInput( self, img, circles, color ):
		if circles is not None:
   
			detected_circles = np.uint16(np.around(circles))

			for pt in detected_circles[0, :]:
				a, b, r = pt[0], pt[1], pt[2]

				if self.getColor( img, a, b, r ) == color:
					return [a, b, r]

		return None

	def locateCircle( self, images, color, draw = False ):
		if images is None:
			return None

		m,n,_ = images[0].shape
		# print m,n
		circle_left = self._locateCircle( images[0] )
		circle_right = self._locateCircle( images[1] )

		r_max = 0
		c_left = self._filterInput( images[0], circle_left, color )
		c_right = self._filterInput( images[1], circle_right, color )

		return c_left, c_right

	def getStimulusPositionAndSize(self, images, color):
		c_left, c_right = self.locateCircle( images, color )
		stim_x = None
		stim_y = None
		object_size = 0

		if c_left is None and c_right is None:
			return stim_x, stim_y, object_size

		n,m,_ = images[0].shape

		sigma = m/3.0
		gl = lambda x: np.exp(-((float(x) - 0)**2)/(2*sigma**2))
		gc = lambda x: np.exp(-((-float(x) + m)**2)/(2*sigma**2))
		pl_l, pl_c, pr_c, pr_r = [0]*4
		y_pos = n/2.0
		# left image
		if c_left is not None:
			pl_l = gl( c_left[0] )
			pl_c = gc( c_left[0] )
			object_size = c_left[2]
			y_pos = c_left[1]
		# right image
		if c_right is not None:
			pr_c = gl( c_right[0] )
			pr_r = gc( c_right[0] )
			object_size = c_right[2]
			y_pos = c_right[1]

		if pl_l > pl_c:
			stim_x = -1

		elif pr_r > pr_c:
			stim_x = 1
		else:
			stim_x = 0

		print "y_pos: ", y_pos, ", thr: ", 2*n/3.0
		if y_pos < n/3.0:
			stim_y = -1
		elif y_pos > 2*n/3.0:
			stim_y = 1
		else:
			stim_y = 0

		return stim_x, stim_y, object_size

class ActionSystem:
	def __init__( self, robot ):
		self.robot = robot
		self.angle = 34

	def wag( self ):
		self.robot.tailWag()

	def follow( self, location, xi ):
		forces = [0.0, 0.0]

		if location is None:
			return

		if xi < 0.5:
			return

		if location == -1:
			forces[1] = 1.0*xi
		elif location == 1:
			forces[0] = 1.0*xi
		elif location == 0:
			forces = [xi, xi]

		self.robot.move( forces )

	def track( self, location_y, xi ):
		if location_y is None:
			return

		print "loc_y: ", location_y

		if location_y == 1 and self.angle == 34:
			self.angle = 60
		elif location_y == -1  and self.angle == 60:
			self.angle = 34
		elif location_y == -1  and self.angle == 34:
			self.angle = 1
		elif location_y == 1  and self.angle == 1:
			self.angle = 34

		self.robot.moveHead( self.angle )

	def search( self, dir ):
		v = dir*0.1
		forces = [0, v]
		self.robot.move( forces )

	def backtrack( self ):
		v = -0.2
		forces = [v, v]
		self.robot.move( forces )

	def shine( self, color, xi ):
		idxs = []
		if color == 'red':
			idxs = range(int(xi*6))
			c = 0xFFFF0000
		elif color == 'green':
			idxs = [6 - x - 1 for x in range(int(xi*6))]
			c = 0xFF00FF00
		else:
			c = 0x00000000

		self.robot.shine( c, idxs )


	def stop( self ):
		self.robot.move([0,0])

class MotivationalSystem(object):
	def __init__(self, actions, perception):
		self.actions = actions
		self.perception = perception
		self.object_size = 0
		self.s = 0
		self.elapsed = 0

	def perceive(self, images, xi):
		pass

	def express( self, xi ):
		pass

	def behave( self, xi ):
		pass

	def iterate( self, images, xi ):
		self.perceive( images, xi )
		self.express( xi )
		self.behave( xi )

		r = self.perception.isClose( images, self.object_size )

		if r == 1 and self.s == 1 and xi > 0.5:
			self.actions.stop()
			self.s = 2
			self.object_size = 0

		return r

class RedMotivationalSystem(MotivationalSystem):
	def __init__(self, actions, perception):
		super(RedMotivationalSystem, self).__init__(actions, perception)
		self.stim_x = None
		self.stim_y = None

	def perceive(self, images, xi):
		if images is None:
			self.stim_x = None
			self.stim_y = None
			return

		self.stim_x, self.stim_y, self.object_size = self.perception.getStimulusPositionAndSize(images, 'red')

	def express( self, xi ):
		self.actions.shine( 'red', xi )

		if self.s == 2:
			self.actions.wag()

	def behave( self, xi ):
		if self.stim_x is None and self.s != 0:
			self.elapsed += 1

			print self.elapsed

			if self.elapsed > 200:
				self.s = 0
				self.elapsed = 0
		elif  self.stim_x is not None and self.s == 0:
			self.s = 1
			self.elapsed = 0
			self.actions.stop()

		if self.s == 0:
			if xi > 0.5:
				self.actions.search(1)
		elif self.s == 1:
			self.actions.track( self.stim_y, xi )
			self.actions.follow( self.stim_x, xi )
			self.s = 1

class GreenMotivationalSystem(MotivationalSystem):
	def __init__(self, actions, perception):
		super(GreenMotivationalSystem, self).__init__(actions, perception)
		self.stim_x = None
		self.stim_y = None
		self.elapsed = 0

	def perceive(self, images, xi):
		if images is None:
			self.stim_x = None
			self.stim_y = None
			return

		self.stim_x, self.stim_y, self.object_size = self.perception.getStimulusPositionAndSize(images, 'green')

	def express( self, xi ):
		self.actions.shine( 'green', xi )

		if self.s == 2:
			self.actions.wag()

	def behave( self, xi ):
		if self.stim_x is None and self.s != 0:
			self.elapsed += 1

			if self.elapsed > 200:
				self.s = 0
				self.elapsed = 0

		elif  self.stim_x is not None and self.s == 0:
			self.s = 1
			self.elapsed = 0
			self.actions.stop()

		if self.s == 0:
			if xi > 0.5:
				self.actions.search(-1)
		elif self.s == 1:
			self.actions.track( self.stim_y, xi )
			self.actions.follow( self.stim_x, xi )
			self.s = 1


class AudioMotivationalSystem(MotivationalSystem):
	def __init__(self, actions, perception):
		super(AudioMotivationalSystem, self).__init__(actions, perception)
		self.stim_x = None
		# self.stim_y = None

	def perceive(self, audio, xi):
		if audio is None:
			self.stim_x = None
			# self.stim_y = None
			return

		# TODO: Make this tidier, take multiple freqs
		self.stim_x = self.perception.locate_frequencies(audio)[0]
		if self.stim_x > 0:
			self.stim_x = 1
		elif self.stim_x < 0:
			self.stim_x = -1
		else:
			self.stim_x = 0

		print('Audio direction: ' + str(self.stim_x))
		# self.stim_x, self.stim_y, self.object_size = self.perception.getStimulusPositionAndSize(images, 'red')

	def express(self, xi):
		self.actions.shine('red', xi)

		# if self.s == 2:
		# 	self.actions.wag()

	def behave(self, xi):
		self.actions.follow(self.stim_x, xi)
		print("reward:" + str(xi))

	def iterate( self, audio, xi ):
		self.perceive( audio, xi )
		self.express( xi )
		self.behave( xi )

		r = self.perception.isClose(audio)
		#
		# if r == 1 and self.s == 1 and xi > 0.5:
		# 	self.actions.stop()
		# 	self.s = 2
		# 	self.object_size = 0

		return r


class HypothalamusController:
	def __init__( self, robot ):
		self.s0 = np.array([0.5, 1.0, 0.0]);
		self.state = self.s0
		self.robot = robot
		self.perception = VisualPerception()

		self.audio_perception = AudioPerception()

		self.actions = ActionSystem( robot )
		self.mot_green = GreenMotivationalSystem(self.actions, self.perception)
		self.mot_red = RedMotivationalSystem( self.actions, self.perception)

		self.mot_audio = AudioMotivationalSystem( self.actions, self.audio_perception)

		self.preferred = {'E1' : 1.0, 'E2': 1.0, 'Rho': None}
		self.variables = {'DGreen': 0.0, 'DRed': 0.0, 'mu_green': 0.0, 'mu_red': 0.0}
		self.reward_r = 0
		self.reward_g = 0
		self.diff_heat = 0.2
		self.images = None
		self.audio = 0

		# self.fig, self.ax = plt.subplots(1,2)

		self.initMaps()

	def initMaps( self ):
		# Drive map green
		G_g = 2.0
		sigma_g = 10.0
		x0 = 0.0
		N_green = lambda x: G_g/(1 + np.exp(-sigma_g*(x - x0))) - G_g/2.0
		U_green = lambda x: x**3
		self.D_green = lambda x: U_green( N_green(x) )
		# Drive map Red
		G_r = 2.0
		sigma_r = 10.0
		x0 = 0.0
		N_red = lambda x: G_r/(1 + np.exp(-sigma_r*(x - x0))) - G_r/2.0
		U_red = lambda x: x**3
		self.D_red = lambda x: U_red( N_red(x) )
		# Incentive maps
		sigma_c = 0.5
		self.xi = lambda x,x0: np.exp(-(x - x0)**2/(2*sigma_c**2))

	def drive_map( self, E1, E2, reward_g, reward_r ):
		
		# Computing physiological state
		alpha = 0.5
		beta = 0.5
		G = 0.5

		print "reward r: ", reward_r, ", Reward g: ", reward_g

		dE1 = -alpha*G + (reward_g*50 if E1 < 1 else 0.0)
		dE2 = -beta*G + (reward_r*50 if E2 < 1 else 0.0)

		# print "dE1: ", dE1, ", dE2: ", dE2
		# Setting up the drives
		self.variables['dGreen'] = self.D_green( E1 - self.preferred['E1'] )
		self.variables['dRed'] = self.D_red( E2 - self.preferred['E2'] )

		return dE1, dE2

	def motivation_map( self, h, rho ):
		L = 20.0

		# Competition parameters
		a = np.abs(self.variables['dGreen'])
		b = np.abs(self.variables['dRed'])


		U = lambda rho: (1.0/4.0)*rho**2*(1 - rho)**2 + a*rho**2 + b*(1 - rho)**2
		dU = lambda rho: (1.0/2.0)*(rho*((1-rho)**2 + a) - (1-rho)*(rho**2 + b))

		noise = np.random.normal(loc = 0.0, scale=self.diff_heat)/np.sqrt(h)
		dRho = -L*dU( rho ) + noise

		return dRho

	def incentives_map( self, rho ):
		GREEN_STATE, RED_STATE = 0, 1

		self.variables['mu_green'] = self.xi(rho, GREEN_STATE)
		self.variables['mu_red'] = self.xi(rho, RED_STATE)

		# print "Incentive temp: ", mu_heat
		# print "Incentive food: ", mu_food

		return self.variables['mu_green'], self.variables['mu_red']

	def dmap( self, t, h, reward_g, reward_r ):
		E1 = self.state[0]
		E2 = self.state[1]
		rho = self.state[2]

		dE1, dE2 = self.drive_map(E1, E2, reward_g, reward_r)
		dRho = self.motivation_map( h, rho )
		mu_green, mu_red = self.incentives_map( rho )

		return np.array([dE1, dE2, dRho]), mu_green, mu_red

	def step( self, im, h, t, au ):
		self.images = im
		self.audio = au
		# Step brain
		# print "Reward r: ", self.reward_r, ", reward G: ", self.reward_g
		di, xi_g, xi_r = self.dmap( t, h, int(self.reward_g), int(self.reward_r) )

		# print "mu_r: ", xi_r, ", mu_g: ", xi_g

		if self.state is not None and len(di) > 0:
			self.state = self.state + h*di

			print "state: " ,self.state

			for i in range(len(self.state)):
				if self.state[i] <= 0:
					self.state[i] = 0.0

		# Iterate motivational systems
		self.reward_r = self.mot_red.iterate( im, xi_r )
		self.reward_g = self.mot_green.iterate( im, xi_g )

		# self.reward_g = self.mot_audio.iterate(au, xi_g)

	def plots( self ):
		images = self.images

		if images is None:
			return

		# img = images[0]
		# im = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
		#  # Set up the detector with default parameters.
		# detector = cv2.SimpleBlobDetector_create()

		# # Detect blobs.
		# keypoints = detector.detect(im)
		# print keypoints

		# # Draw detected blobs as red circles.
		# # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
		# im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

		# # Show keypoints
		# cv2.imshow("Keypoints", im_with_keypoints)
		# cv2.waitKey(1)
		# imr =  ep.getEntropy(imr)
		# self.ax[0].imshow( images[0])
		# self.ax[1].imshow( images[1] )
