class MotivationalSystem(object):
	def __init__(self, actions, perception):
		self.actions = actions
		self.perception = perception
		self.object_size = 0
		self.s = 0
		self.elapsed = 0

	def perceive(self, images, xi):
		pass

	def express(self, xi):
		pass

	def behave(self, xi):
		pass

	def iterate(self, images, xi):
		self.perceive(images, xi)
		self.express(xi)
		self.behave(xi)

		r = self.perception.isClose(images, self.object_size)

		if r == 1 and xi > 0.5:
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

	def express(self, xi):
		self.actions.shine('red', xi)

		if self.s == 2:
			self.actions.wag()

	def behave(self, xi):
		if self.stim_x is None and self.s != 0:
			self.elapsed += 1

			if self.elapsed > 20:
				self.s = 0
				self.elapsed = 0

		elif self.stim_x is not None and self.s == 0:
			self.s = 1
			self.elapsed = 0
			self.actions.stop()

		if self.s == 0:
			if xi > 0.5:
				self.actions.search(-1)
		elif self.s == 1:
			self.actions.follow(self.stim_x, xi)
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

	def express(self, xi):
		self.actions.shine('green', xi)

		if self.s == 2:
			self.actions.wag()

	def behave(self, xi):
		if self.stim_x is None and self.s != 0:
			self.elapsed += 1

			if self.elapsed > 20:
				self.s = 0
				self.elapsed = 0

		elif self.stim_x is not None and self.s == 0:
			self.s = 1
			self.elapsed = 0
			self.actions.stop()

		if self.s == 0:
			if xi > 0.5:
				self.actions.search(1)
		elif self.s == 1:
			self.actions.follow(self.stim_x, xi)
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

		thres = 0.5
		# TODO: Make this tidier, take multiple freqs
		self.stim_x = self.perception.locate_frequencies(audio)[0]

		if self.stim_x > thres:
			self.stim_x = -1
		elif self.stim_x < -thres:
			self.stim_x = 1
		else:
			self.stim_x = 0

		print('Audio direction: ' + str(self.stim_x))

	# self.stim_x, self.stim_y, self.object_size = self.perception.getStimulusPositionAndSize(images, 'red')

	def express(self, xi):
		self.actions.shine('green', xi)

		if self.s == 2:
			self.actions.wag()

	def behave(self, xi):
		if self.s == 1:
			if self.stim_x != 0:
				self.actions.follow(self.stim_x, xi)
			print("reward:" + str(xi))
        

	def iterate(self, audio, xi):
		self.perceive(audio, xi)
		self.express(xi)
		self.behave(xi)

		rs = self.perception.isClose(audio)

		r = rs[0]
		# if r == 1:
		# 	self.actions.stop()

		if r == 1 and xi > 0.5:
			self.actions.stop()
			self.s = 2
			self.object_size = 0
		else:
			self.s = 1

		return r


class TagMotivationalSystem(MotivationalSystem):
	def __init__(self, actions, perception, tag_id, colour):
		super(TagMotivationalSystem, self).__init__(actions, perception)
		self.stim_x = None
		self.stim_y = None
		self.elapsed = 0

		self.atp = AprilTagPerception(size=8.2)

		self.tag_id = tag_id
		self.colour = colour

		self.distance = None

	def perceive(self, images, xi):
		def tag_distance_and_location(tags, tag_id):
			distance = [t.distance for t in tags if t.id == tag_id]
			location = [t.centre for t in tags if t.id == tag_id]
			apparent_size = [t.apparent_size for t in tags if t.id == tag_id]

			if distance:
				return distance[0], location[0], apparent_size
			else:
				return None, None, None

		def annotate_image(image, tags, colour, tag_id):
			[self.atp.draw_box(tag=t, image=image, colour=colour) for t in tags if t.id == tag_id]
			[self.atp.draw_center(tag=t, image=image, colour='magenta') for t in tags if t.id == tag_id]

		# Main
		# FIXME: Do both eyes at once
		caml = images[0]
		camr = images[1]
		tags_left = self.atp.detect_tags(caml)
		tags_right = self.atp.detect_tags(camr)

		distance_left = location_left = distance_right = location_right = self.distance = None

		try:
			[distance_left, location_left, apparent_size_left] = tag_distance_and_location(tags=tags_left, tag_id=self.tag_id)
			# annotate_image(image=caml, tags=tags_left, colour=self.colour, tag_id=self.tag_id)
		# If no left tags exist
		except TypeError:
			distance_left = location_left = apparent_size_left = None

		try:
			[distance_right, location_right, apparent_size_right] = tag_distance_and_location(tags=tags_right, tag_id=self.tag_id)
			# annotate_image(image=camr, tags=tags_right, colour=self.colour, tag_id=self.tag_id)
		# If no right tags exist
		except TypeError:
			distance_right = location_right = apparent_size_right = None

		try:
			c_left = list(location_left) + apparent_size_left
		# If at least one attribute is missing
		except TypeError:
			c_left = None

		try:
			c_right = list(location_right) + apparent_size_right
		except TypeError:
			# If at least one attribute is missing
			c_right = None

		# if self.colour == 'red':
		# 	print('debug here')

		self.stim_x, self.stim_y, self.object_size = self.perception.getStimulusPositionAndSize(
			images, self.colour, c_left=c_left, c_right=c_right
		)

		# if self.colour == 'green':
		# 	print('debug here')

		# Get distance
		# TODO: Tidy this, is a mess
		if distance_left is not None and distance_right is not None:
			self.distance = np.min([distance_left, distance_right])
		elif distance_left is not None:
			self.distance = distance_left
		elif distance_right is not None:
			self.distance = distance_right

		try:
			print('Distance to {}: {:.1f}cm'.format(self.colour, self.distance))
			# print('debug here')
		except TypeError:
			pass

	def iterate(self, images, xi):
		self.perceive(images, xi)
		self.express(xi)
		self.behave(xi)


		# r = self.perception.isClose(images, self.object_size)

		try:
			if self.distance < 30 and xi > 0.5:
				self.actions.stop()
				self.s = 2
				self.object_size = 0

				print("I'm close!")

				r = True
			else:
				r = False
		# If distance is 'None'
		except TypeError:
			r = False

		return r

	def express(self, xi):
		self.actions.shine(self.colour, xi)

		if self.s == 2:
			self.actions.wag()

	def behave(self, xi):
		print('BEHAVE   {}'.format(self.colour))
		if self.distance is not None:
			print('Distance:    {}'.format(self.distance))
		print('stim_x:  {}'.format(self.stim_x))
		print('s:       {}'.format(self.s))
		print('xi:      {}'.format(xi))

		# if self.colour == 'red' and self.stim_x is not None:
		# 	print('debug here')

		if self.stim_x is None and self.s != 0:
			print('Elapsed: {}'.format(self.elapsed))
			self.elapsed += 1

			if self.elapsed > 20:
				print('Setting s to 0')
				self.s = 0
				self.elapsed = 0

		elif self.stim_x is not None and self.s == 0:
			print('➤ STOP')
			self.s = 1
			self.elapsed = 0
			self.actions.stop()

		if self.s == 0:
			if xi > 0.5:
				print('➤ SEARCH')
				self.actions.search_new(direction='right')
		elif self.s == 1:
			print('➤ FOLLOW')
			self.actions.follow(self.stim_x, xi)
			self.s = 1

		print('')