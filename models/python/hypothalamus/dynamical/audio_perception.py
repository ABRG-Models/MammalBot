import numpy as np

# From http://labs.consequentialrobotics.com/miro-e/docs/index.php?page=Technical_Interfaces_ROS
SAMP_PER_BLOCK = 500        # Sample rate of 20kHz and packages arriving at ~40Hz == 500 samples per package
BLOCK_RATE = 40             # Package arrival rate
MICS = 4


class AudioPerception:
	def __init__(self):
		# Audio frequency parameters
		self.frequencies = [440, 600]       # Frequencies to detect (multiples of 40 recommended)
		self.buffer_length = 10             # At 40Hz a buffer of 20 == 0.5s
		self.f_thresh = 0.5                  # Minimum power at which to register frequency detection

		# FFT array positions of defined frequencies
		self.f_pos = [int(frq / BLOCK_RATE) for frq in self.frequencies]
		self.f_buffer = np.zeros((len(self.frequencies), self.buffer_length, MICS))
		self.f_mean = np.zeros((len(self.frequencies), MICS))

	@staticmethod
	def shape_data(data):
		# Reshape data (from node_detect_audio_engine.py)
		data = np.asarray(data, 'float32') * (1.0 / 32768.0)
		data = data.reshape((MICS, SAMP_PER_BLOCK))

		return data

	def do_fft(self, data):
		# Perform Fast Fourier Transform on audio sample from each microphone, dropping mirrored and imaginary portions
		fft_data = [np.abs(np.fft.fft(mic))[: SAMP_PER_BLOCK / 2] for mic in data]

		# Get power of specified frequencies from each microphone
		# Mic data is ordered [LEFT, RIGHT, CENTRE, TAIL]
		freq_power = np.array([
			[fft_data[mic][self.f_pos[frq]] for mic, _ in enumerate(fft_data)]
			for frq, _ in enumerate(self.frequencies)
		])

		return freq_power

	def locate_frequencies(self, data):
		# # Reshape data (from node_detect_audio_engine.py)
		# data = np.asarray(data, 'float32') * (1.0 / 32768.0)
		# data = data.reshape((MICS, SAMP_PER_BLOCK))
		data = self.shape_data(data)

		# # Perform Fast Fourier Transform on audio sample from each microphone, dropping mirrored and imaginary portions
		# fft_data = [np.abs(np.fft.fft(mic))[: SAMP_PER_BLOCK / 2] for mic in data]
		#
		# # Get power of specified frequencies from each microphone
		# # Mic data is ordered [LEFT, RIGHT, CENTRE, TAIL]
		# freq_power = np.array([
		# 	[fft_data[mic][self.f_pos[frq]] for mic, _ in enumerate(fft_data)]
		# 	for frq, _ in enumerate(self.frequencies)
		# ])

		freq_power = self.do_fft(data)

		# Add frequency power from this sample to the buffer and store the mean power to smooth out noise
		for frq, mic in enumerate(freq_power):
			# Shift frequency buffer and append new values
			self.f_buffer[frq] = np.roll(self.f_buffer[frq], 1, axis=0)
			self.f_buffer[frq][0] = freq_power[frq]

			for m, _ in enumerate(mic):
				# Calculate mean power of each frequency across entire buffer
				self.f_mean[frq][m] = np.mean(self.f_buffer[frq][:, m])

		# Get leftwards bias (i.e. positive == left, negative == right) for each frequency if mean values are above threshold
		f_left = np.zeros(len(freq_power))
		for frq, _ in enumerate(freq_power):
			if self.f_mean[frq][0] > self.f_thresh and self.f_mean[frq][1] > self.f_thresh:
				# Leftwards bias is just mean left power - mean right power
				f_left[frq] = self.f_mean[frq][0] - self.f_mean[frq][1]
			else:
				f_left[frq] = None

		# TODO: Get front/back bias from centre/tail mics (needs further calibration due to differing sensitivities)

		return f_left

	def isClose(self, data):
		data = self.shape_data(data)

		freq_power = self.do_fft(data)

		# PICKED RANDOM NUMBER FOR POWER LEVEL
		# TODO: Update to play nice with more frequencies
		print('freq power: ' + str(freq_power[0][3]))
		if freq_power[0][3] > 5:
			return 1
		else:
			return 0




