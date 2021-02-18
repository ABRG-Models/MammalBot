from contextlib import suppress
from dataclasses import dataclass, InitVar, field
import math
import model_templates
import numpy as np
import sys


@dataclass
class BasalGanglia:
	channels: int                                   # Number of input channels
	lh      : InitVar[bool]                         # Use lateral hypothalamus?
	trn     : InitVar[bool]                         # Use thalamic reticular nucleus?
	model   : dict = field(default_factory=dict)    # Model data structure

	# Artificial parameters
	# Gain
	k = 25
	dt = 0.01
	decay_constant = math.exp(-k * dt)
	# Slope
	m = 1  # Humphries & Gurney (2002)

	def __post_init__(self, lh, trn):
		# Initialise model from defaults
		self.model['Ventral'] = model_templates.HumphriesPrescott2010
		self.model['Dorsal'] = model_templates.HumphriesGurney2002

		# Input, activity and output arrays
		self.input = np.zeros(self.channels)
		for reg in self.model.keys():
			for pop in self.model[reg].keys():
				self.model[reg][pop].a = np.zeros(self.channels)
				self.model[reg][pop].o = np.zeros(self.channels)

		# Lesion TRN
		if not trn:
			self.model['Dorsal'].pop('TRN')
			self.model['Dorsal']['Thal'].w.pop('TRN')

		# Add lateral hypothalamus
		if lh:
			self.lateral_hypothalamus = {
				'Approach': model_templates.Population(
					name='Approach pathway',
					e=None,
				),
				'Avoid': model_templates.Population(
					name='Avoid pathway',
					e=None,
				)
			}

			self.model['Dorsal']['DA'].w['LH'] = {
				'Approach': 1,
				'Avoid'   : -1
			}

		# TODO: Make this an option
		# Bias weights towards external inputs
		self.model['Dorsal']['dMSN'].w = {
			'Inp': 0.75,
			'Ctx': 0.25,
		}

		self.model['Dorsal']['iMSN'].w = {
			'Inp': 0.75,
			'Ctx': 0.25,
		}

		# # Store input and output history
		# # TODO: Use built-in logging module to do this?
		# # TODO: Saving data should occur outside of the BG model class
		# if save:
		# 	self.data = {'Input': [['Input CH' + str(ch + 1)] for ch in range(self.channels)]}
		# 	self.data.update({
		# 		reg: {
		# 			pop: [[reg + ' ' + pop + ' CH' + str(ch + 1)] for ch in range(self.channels)]
		# 			for pop in self.model[reg].keys()
		# 		} for reg in self.model.keys()
		# 	})

	@staticmethod
	def ramp(a, e, m, /):
		# Piecewise linear squashing function
		# Eq. 2 from Gurney, Prescott, & Redgrave (2001b)
		ramp_output = np.zeros(len(a))

		a_min = np.where(a < e)
		a_rmp = np.where((a >= e) & (a <= 1 / m + e))
		a_max = np.where(a > 1 / m + e)

		ramp_output[a_min] = 0
		ramp_output[a_rmp] = m * (a[a_rmp] - e)
		ramp_output[a_max] = 1

		return ramp_output

	def activation(self, dst_region, dst_pop, /):
		u = 0

		# Compute activation by all input sources
		for src_name, src_weight in self.model[dst_region][dst_pop].w.items():
			if src_name != 'Inp':

				# Special case: Input from remote region
				if src_name in self.model.keys():
					src_region = src_name

					for remote_name, remote_weight in self.model[dst_region][dst_pop].w[src_region].items():
						remote_output = self.model[src_region][remote_name].o
						u = u + remote_weight * remote_output

				# Special case: Input from lateral hypothalamus
				elif src_name == 'LH':
					for lh_path, lh_weight in self.model[dst_region][dst_pop].w['LH'].items():
						try:
							lh_output = self.lateral_hypothalamus[lh_path].o
							u = u + lh_weight * lh_output
						except AttributeError:
							print('No lateral hypothalamus \'' + lh_path + '\' input provided')

				# General case: Input from local region
				else:
					src_region = dst_region
					src_output = self.model[src_region][src_name].o

					# Special case: STN projections are not channel specific
					if src_name == 'STN':
						u = u + src_weight * sum(src_output)

					# Special case: TRN
					elif src_name == 'TRN':
						u = u + (
							src_weight['within']
							* src_output
							+ src_weight['between']
							* (sum(src_output) * np.ones(len(src_output)) - src_output)
						)

					# General case: Sum of weighted inputs
					else:
						u = u + src_weight * src_output

			# Special case: Model inputs
			else:
				u = u + src_weight * self.input

		# Dopamine modifier
		# TODO: Add support for remote dopamine modulation
		# Prefer EAFP to LBYL: https://stackoverflow.com/questions/610883/how-to-know-if-an-object-has-an-attribute-in-python/610923#610923
		# Suppress KeyError: We expect failures in regions with no DA population
		# Suppress TypeError: We expect failures in populations with no DA modulation (where da=None)
		with suppress(KeyError, TypeError):
			# Eq. 11 from Humphries & Gurney (2002)
			u = u * (1 + self.model[dst_region]['DA'].o * self.model[dst_region][dst_pop].da)

		return u

	def step(self, c, /, **kwargs):
		# BG inputs
		self.input = c

		# LH inputs
		try:
			# Suppress KeyError: No LH inputs is not a problem here
			with suppress(KeyError):
				self.lateral_hypothalamus['Approach'].o = kwargs.pop('lh_approach')
			with suppress(KeyError):
				self.lateral_hypothalamus['Avoid'].o = kwargs.pop('lh_avoid')
		except AttributeError:
			print('❗ Model received lateral hypothalamus inputs but was instantiated with \'lh=False\'')
			sys.exit(1)

		# # TODO: Change to use logging and/or make check dependent on data struct existing
		# if self.save:
		# 	for ch, val in enumerate(input):
		# 		self.data['Input'][ch].append(val.item())

		for reg in self.model.keys():
			for pop in self.model[reg].keys():
				# Post-synaptic potential (total afferent input)
				self.model[reg][pop].u = self.activation(reg, pop)

				# Leaky integrator activation
				# Eq. 1 from Gurney, Prescott, & Redgrave (2001b)
				self.model[reg][pop].a = (self.model[reg][pop].a - self.model[reg][pop].u) \
				                            * self.decay_constant \
				                            + self.model[reg][pop].u

				# Outputs
				self.model[reg][pop].o = self.ramp(self.model[reg][pop].a, self.model[reg][pop].e, self.m)

				# # Store output data
				# if self.save:
				# 	for ch, val in enumerate(self.model[reg][pop]['o']):
				# 		# item() converts numpy.float64 to native Python float
				# 		self.data[reg][pop][ch].append(val.item())
