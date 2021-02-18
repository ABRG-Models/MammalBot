from contextlib import suppress
from dataclasses import dataclass, InitVar, field
import math
import numpy as np
import sys
from typing import Optional


@dataclass
class Population:
	name: str                                   # Name
	e   : Optional[float]                       # Activation offset (negative implies tonically active)
	da  : Optional[float] = None                # Dopamine modulation (negative implies inhibitory effect)
	w   : dict = field(default_factory=dict)    # Input weights


HumphriesGurney2002 = {
	'dMSN': Population(
		name='Direct-pathway MSNs',
		e=0.2,
		da=1,   # dMSN DA is excitatory
		w={
			'Inp': 0.5,
			'Ctx': 0.5,
		},
	),
	'iMSN': Population(
		name='Indirect-pathway MSNs',
		e=0.2,
		da=-1,  # iMSN DA is inhibitory
		w={
			'Inp': 0.5,
			'Ctx': 0.5,
		},
	),
	'STN' : Population(
		name='Subthalamic nucleus',
		e=-0.25,
		w={
			'Inp': 0.5,
			'Ctx': 0.5,
			'GPe': -1,
		},
	),
	'GPe' : Population(
		name='Globus pallidus (external)',
		e=-0.2,
		w={
			'iMSN': -1,
			'STN' : 0.8,
		},
	),
	'SNr' : Population(
		name='Substantia nigra pars reticulata',
		e=-0.2,
		w={
			'dMSN': -1,
			'STN' : 0.8,
			'GPe' : -0.4,
		},
	),
	'Thal': Population(
		name='Thalamus',
		e=0,
		w={
			'SNr': -1,
			'Ctx': 1,
			'TRN': {
				'between': -0.7,
				'within' : -0.1,
			}
		},
	),
	'TRN': Population(
		name='Thalamic reticular nucleus',
		e=0,
		w={
			'Ctx' : 1,
			'Thal': 1,
		}
	),
	'Ctx' : Population(
		name='Cortex',
		e=0,
		w={
			'Inp' : 1,
			'Thal': 1,
		},
	),
	'DA'  : Population(
		name='Dopamine output',
		e=-0.2,
	),
}

HumphriesPrescott2010 = {
	'dMSN': Population(
		name='Direct-pathway MSNs',
		e=0.2,
		w={'Inp': 1}
	),
	'iMSN': Population(
		name='Indirect-pathway MSNs',
		e=0.2,
		w={'Inp': 1},
	),
	'Pal' : Population(
		name='Pallidum',
		e=-0.2,
		w={'iMSN': -1}
	),
	'PPn' : Population(
		name='Pedunculopontine nucleus',
		e=-0.15,
		w={'Pal': -0.5}
	),
	'VTA' : Population(
		name='Ventral tegmental area',
		e=-0.075,
		w={
			'dMSN': -1,
			'Pal' : -0.5,
			'PPn' : 1,
		},
	),
}


#########
### TESTING
#########


# @dataclass
# # TODO: Implement as non-dataclass so I can remove TRN
# class BasalGangliaNew:
# 	channels: int = 3                   # Number of input channels
# 	lh      : InitVar[bool] = False     # Use lateral hypothalamus?
# 	trn     : InitVar[bool] = False     # Use thalamic reticular nucleus?
#
# 	# Artificial parameters
# 	# Gain
# 	k = 25
# 	dt = 0.01
# 	decay_constant = math.exp(-k * dt)
# 	# Slope
# 	m = 1  # Humphries & Gurney (2002)
#
# 	model = {
# 		'Ventral': HumphriesPrescott2010,
# 		'Dorsal' : HumphriesGurney2002,
# 	}
#
# 	# Input, activity and output arrays
# 	input = np.zeros(channels)
# 	for reg in model.keys():
# 		for pop in model[reg].keys():
# 			model[reg][pop].a = np.zeros(channels)
# 			model[reg][pop].o = np.zeros(channels)
#
# 	def __post_init__(self, lh, trn):
# 		# Lesion TRN
# 		if not trn:
# 			self.model['Dorsal'].pop('TRN')
# 			self.model['Dorsal']['Thal'].w.pop('TRN')
#
# 		# Add lateral hypothalamus
# 		if lh:
# 			self.lateral_hypothalamus = {
# 				'Approach': Population(
# 					name='Approach pathway',
# 					e=None,
# 				),
# 				'Avoid': Population(
# 					name='Avoid pathway',
# 					e=None,
# 				)
# 			}
#
# 			self.model['Dorsal']['DA'].w['LH'] = {
# 				'Approach': 1,
# 				'Avoid'   : -1
# 			}
#
# 		# TODO: Make this an option
# 		# Bias weights towards external inputs
# 		self.model['Dorsal']['dMSN'].w = {
# 			'Inp': 0.75,
# 			'Ctx': 0.25,
# 		}
#
# 		self.model['Dorsal']['iMSN'].w = {
# 			'Inp': 0.75,
# 			'Ctx': 0.25,
# 		}
#
# 		# # Store input and output history
# 		# # TODO: Use built-in logging module to do this?
# 		# # TODO: Saving data should occur outside of the BG model class
# 		# if save:
# 		# 	self.data = {'Input': [['Input CH' + str(ch + 1)] for ch in range(self.channels)]}
# 		# 	self.data.update({
# 		# 		reg: {
# 		# 			pop: [[reg + ' ' + pop + ' CH' + str(ch + 1)] for ch in range(self.channels)]
# 		# 			for pop in self.model[reg].keys()
# 		# 		} for reg in self.model.keys()
# 		# 	})
#
# 	@staticmethod
# 	def ramp(a, e, m, /):
# 		# Piecewise linear squashing function
# 		# Eq. 2 from Gurney, Prescott, & Redgrave (2001b)
# 		ramp_output = np.zeros(len(a))
#
# 		a_min = np.where(a < e)
# 		a_rmp = np.where((a >= e) & (a <= 1 / m + e))
# 		a_max = np.where(a > 1 / m + e)
#
# 		ramp_output[a_min] = 0
# 		ramp_output[a_rmp] = m * (a[a_rmp] - e)
# 		ramp_output[a_max] = 1
#
# 		return ramp_output
#
# 	def activation(self, dst_region, dst_pop, /):
# 		u = 0
#
# 		# Compute activation by all input sources
# 		for src_name, src_weight in self.model[dst_region][dst_pop].w.items():
# 			if src_name != 'Inp':
#
# 				# Special case: Input from remote region
# 				if src_name in self.model.keys():
# 					src_region = src_name
#
# 					for remote_name, remote_weight in self.model[dst_region][dst_pop].w[src_region].items():
# 						remote_output = self.model[src_region][remote_name].o
# 						u = u + remote_weight * remote_output
#
# 				# Special case: Input from lateral hypothalamus
# 				elif src_name == 'LH':
# 					for lh_path, lh_weight in self.model[dst_region][dst_pop].w['LH'].items():
# 						try:
# 							lh_output = self.lateral_hypothalamus[lh_path].o
# 							u = u + lh_weight * lh_output
# 						except AttributeError:
# 							print('No lateral hypothalamus \'' + lh_path + '\' input provided')
#
# 				# General case: Input from local region
# 				else:
# 					src_region = dst_region
# 					src_output = self.model[src_region][src_name].o
#
# 					# Special case: STN projections are not channel specific
# 					if src_name == 'STN':
# 						u = u + src_weight * sum(src_output)
#
# 					# Special case: TRN
# 					elif src_name == 'TRN':
# 						u = u + (
# 							src_weight['within']
# 							* src_output
# 							+ src_weight['between']
# 							* (sum(src_output) * np.ones(len(src_output)) - src_output)
# 						)
#
# 					# General case: Sum of weighted inputs
# 					else:
# 						u = u + src_weight * src_output
#
# 			# Special case: Model inputs
# 			else:
# 				u = u + src_weight * self.input
#
# 		# Dopamine modifier
# 		# TODO: Add support for remote dopamine modulation
# 		# Prefer EAFP to LBYL: https://stackoverflow.com/questions/610883/how-to-know-if-an-object-has-an-attribute-in-python/610923#610923
# 		# Suppress KeyError: We expect failures in regions with no DA population
# 		# Suppress TypeError: We expect failures in populations with no DA modulation (where da=None)
# 		with suppress(KeyError, TypeError):
# 			# Eq. 11 from Humphries & Gurney (2002)
# 			u = u * (1 + self.model[dst_region]['DA'].o * self.model[dst_region][dst_pop].da)
#
# 		return u
#
# 	def step(self, c, /, **kwargs):
# 		# BG inputs
# 		self.input = c
#
# 		# LH inputs
# 		try:
# 			# Suppress KeyError: No LH inputs is not a problem here
# 			with suppress(KeyError):
# 				self.lateral_hypothalamus['Approach'].o = kwargs.pop('lh_approach')
# 			with suppress(KeyError):
# 				self.lateral_hypothalamus['Avoid'].o = kwargs.pop('lh_avoid')
# 		except AttributeError:
# 			print('❗ Model received lateral hypothalamus inputs but was instantiated with \'lh=False\'')
# 			sys.exit(1)
#
# 		# # TODO: Change to use logging and/or make check dependent on data struct existing
# 		# if self.save:
# 		# 	for ch, val in enumerate(input):
# 		# 		self.data['Input'][ch].append(val.item())
#
# 		for reg in self.model.keys():
# 			for pop in self.model[reg].keys():
# 				# Post-synaptic potential (total afferent input)
# 				self.model[reg][pop].u = self.activation(reg, pop)
#
# 				# Leaky integrator activation
# 				# Eq. 1 from Gurney, Prescott, & Redgrave (2001b)
# 				self.model[reg][pop].a = (self.model[reg][pop].a - self.model[reg][pop].u) \
# 				                            * self.decay_constant \
# 				                            + self.model[reg][pop].u
#
# 				# Outputs
# 				self.model[reg][pop].o = self.ramp(self.model[reg][pop].a, self.model[reg][pop].e, self.m)
#
# 				# # Store output data
# 				# if self.save:
# 				# 	for ch, val in enumerate(self.model[reg][pop]['o']):
# 				# 		# item() converts numpy.float64 to native Python float
# 				# 		self.data[reg][pop][ch].append(val.item())
#
#
# bgtest = BasalGangliaNew(lh=True)
# print(bgtest)
# bgtest.step(np.array([1, 1, 1]), lh_approach=11, lh_avoid=11)
# bgtest.step(np.array([2, 2, 2]))

# LOG_FILENAME = 'example.log'
#
# logging.basicConfig(filename=LOG_FILENAME,level=logging.DEBUG)
#
# # logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)
# logging.debug('This message should appear on the console')
# logging.info('So should this')
# logging.warning('And this, too')