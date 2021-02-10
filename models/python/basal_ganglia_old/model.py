# import copy
import math
import numpy as np


class BasalGanglia(object):
	def __init__(self, channels=3):
		# Set model options
		self.opt = {
			# 'Loop' : True,    # MCx to dorsal striatum loop?
			# 'STR'  : False,   # Striatal intraconnectivity?
			# 'Th_d' : False,   # VLT to dorsal striatum connectivity?

			'Save': False,      # Save all output data?
			'TRN' : True,       # Use TRN?
		}

		# Artificial parameters
		# Gain
		k = 25
		dt = 0.01
		self.decay_constant = math.exp(-k * dt)
		# Slope
		self.m = 1              # Humphries & Gurney (2002)

		# Define BG populations, connections, and weights
		# Weights are defined as 'TO': {'W': {'FROM': <WEIGHT>}}; 'e' is the threshold value epsilon
		self.model = {
			'Ventral': ModelTemplate().hp2010,
			'Dorsal' : ModelTemplate().hg2002,
		}

		if not self.opt['TRN']:
			self.model['Dorsal'].pop('TRN')
			self.model['Dorsal']['Thal']['W'].pop('TRN')

		# Ventral modifications
		# Add DA receptors to ventral striatum
		# self.model['Ventral']['dMSN'].update({'DA': 1})  # Sign modifier (dMSN DA is excitatory)
		# self.model['Ventral']['iMSN'].update({'DA': -1}) # Sign modifier (iMSN DA is inhibitory)
		# self.model['Ventral'].update({
		#     'DA': {
		#         'Name': 'Dopamine neurons',
		#         'e'   : 0,
		#         'W'   : {
		#             'VTA': 1
		#         },
		#     },
		# })

		# # Dorsal modifications
		# self.model['Dorsal']['PPn'] = copy.deepcopy(self.model['Ventral']['PPn'])
		# self.model['Dorsal']['PPn']['W'] = {
		# 	'GPe': -0.5,                # Copy of ventral Pal -> PPn connection
		# 	'SNr': -0.5,                # Copy of ventral Pal -> PPn connection
		# 	'STN': 0.8,                 # Copy of other STN outputs
		# }
		#
		# self.model['Dorsal']['SNc'] = copy.deepcopy(self.model['Ventral']['VTA'])
		# self.model['Dorsal']['SNc']['Name'] = 'Substantia nigra pars compacta'
		# self.model['Dorsal']['SNc']['W'] = {
		# 	'dMSN': -1,                 # Copy of ventral dMSN -> VTA connection
		# 	'GPe' : -0.5,               # Copy of ventral Pal -> VTA connection
		# 	'PPn' : 1,                  # Copy of ventral PPn -> VTA connection
		# }
		#
		#
		#
		# self.model['Dorsal']['dMSN']['W'].update({
		# 	'Ventral': {
		# 		'VTA': 1
		# 	}
		# })
		# self.model['Dorsal']['iMSN']['W'].update({
		# 	'Ventral': {
		# 		'VTA': 1
		# 	}
		# })

		# TODO: Add Dorsal PPN and SNc
		# TODO: Add DA and striatal loop projections
		# TODO: Add DA projections to STN and Pallidum
		# TODO: Differentiate iMSN / dMSN and Ventral / Dorsal inputs
		# TODO: Split VTA into GABA and DA parts (need more excitatory VTA input or evidence of higher tonic rate)
		# TODO: Implement reward signal

		# TODO: Verify this??
		# # Dorsal striatum receiving inputs from 'ventral' cortex reg
		# self.model['Dorsal']['dMSN']['W'] = {
		#     'Inp': 0.5,  # Humphries & Gurney (2002)
		#     'Ctx': 0,
		#     'Ventral': {
		#         'Ctx': 0.5
		#     },
		# }
		#
		# self.model['Dorsal']['iMSN']['W'] = {
		#     'Inp': 0.5,  # Humphries & Gurney (2002)
		#     'Ctx': 0,
		#     'Ventral': {
		#         'Ctx': 0.5
		#     },
		# }


		# MODIFICATIONS AS OF 30/07
		# For simplicity's sake, we will ignore the GABA pop in VTA and directly stimulate dorsal DA neurons for now
		self.approach = {'Weight': 1}
		self.avoid = {'Weight': -1}

		# Store input and output history
		if self.opt['Save']:
			self.data = {'Input': [['Input CH' + str(ch + 1)] for ch in range(channels)]}
			self.data.update({
				reg: {
					pop: [[reg + ' ' + pop + ' CH' + str(ch + 1)] for ch in range(channels)]
					for pop in self.model[reg].keys()
				} for reg in self.model.keys()
			})

		# Input, activity and output arrays
		self.input = np.zeros(channels)
		for reg in self.model.keys():
			for pop in self.model[reg].keys():
				self.model[reg][pop]['a'] = np.zeros(channels)
				self.model[reg][pop]['o'] = np.zeros(channels)

	@staticmethod
	def ramp(a, e, m):
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

	def activation(self, dst_region, dst_pop):
		u = 0

		# Modify DA output in response to LH inputs
		if dst_pop == 'DA' and self.approach['Value'] is not None and self.avoid['Value'] is not None:
			u = self.approach['Value'] * self.approach['Weight'] \
			    + self.avoid['Value'] * self.avoid['Weight']

		# Compute activation by all input sources
		for src_name, src_weight in self.model[dst_region][dst_pop]['W'].items():
			if src_name is not 'Inp':

				# Special case: Input from remote region
				if src_name in self.model.keys():
					src_region = src_name

					for remote_name, remote_weight in self.model[dst_region][dst_pop]['W'][src_region].items():
						remote_output = self.model[src_region][remote_name]['o']
						u = u + remote_weight * remote_output

				# General case: Input from local region
				else:
					src_region = dst_region
					src_output = self.model[src_region][src_name]['o']

					# Special case: STN projections are not channel specific
					if src_name == 'STN':
						u = u + src_weight * sum(src_output)

					# Special case: TRN
					elif src_name == 'TRN':
						u = u + (
							src_weight['within']
							* src_output
							+ src_weight['between']
							* (
								sum(src_output)
								* np.ones(len(src_output))
								- src_output
							)
						)

					# General case: Sum of weighted inputs
					else:
						u = u + src_weight * src_output

			# Special case: Model inputs
			else:
				u = u + src_weight * self.input

		# Dopamine modifier
		# TODO: Add support for remote dopamine modulation
		if 'DA' in self.model[dst_region][dst_pop].keys():
			# Eq. 11 from Humphries & Gurney (2002)
			u = u * (1 + self.model[dst_region]['DA']['o'] * self.model[dst_region][dst_pop]['DA'])

		return u

	def step(self, c, **kwargs):
		# BG inputs
		self.input = c

		# Get LH info externally
		self.approach['Value'] = kwargs.get('LH_APPROACH')
		self.avoid['Value'] = kwargs.get('LH_AVOID')

		if self.opt['Save']:
			for ch, val in enumerate(self.input):
				self.data['Input'][ch].append(val.item())

		for reg in self.model.keys():
			for pop in self.model[reg].keys():
				# Post-synaptic potential
				self.model[reg][pop]['u'] = self.activation(reg, pop)

				# Leaky integrator activation
				# Eq. 1 from Gurney, Prescott, & Redgrave (2001b)
				self.model[reg][pop]['a'] = (self.model[reg][pop]['a'] - self.model[reg][pop]['u']) \
				                            * self.decay_constant \
				                            + self.model[reg][pop]['u']

				# Outputs
				self.model[reg][pop]['o'] = self.ramp(self.model[reg][pop]['a'], self.model[reg][pop]['e'], self.m)

				# Store output data
				if self.opt['Save']:
					for ch, val in enumerate(self.model[reg][pop]['o']):
						# item() converts numpy.float64 to native Python float
						self.data[reg][pop][ch].append(val.item())


class ModelTemplate(object):
	def __init__(self):
		# Humphries & Prescott (2010)
		self.hp2010 = {
			'dMSN': {
				'Name': 'Direct-pathway MSNs',
				'e'   : 0.2,
				'W'   : {'Inp': 1},
			},
			'iMSN': {
				'Name': 'Indirect-pathway MSNs',
				'e'   : 0.2,
				'W'   : {'Inp': 1},
			},
			'Pal' : {
				'Name': 'Pallidum',
				'e'   : -0.2,
				'W'   : {'iMSN': -1}
			},
			'PPn' : {
				'Name': 'Pedunculopontine nucleus',
				'e'   : -0.15,
				'W'   : {'Pal': -0.5}
			},
			'VTA' : {
				'Name': 'Ventral tegmental area',
				'e'   : -0.075,
				'W'   : {
					'dMSN': -1,
					'Pal' : -0.5,
					'PPn' : 1,
				},
			},
		}

		# Humphries & Gurney (2002)
		self.hg2002 = {
			'dMSN': {
				'Name': 'Direct-pathway MSNs',
				'e'   : 0.2,
				'DA'  : 1,              # Sign modifier (dMSN DA is excitatory)
				'W'   : {
					'Inp': 0.5,
					'Ctx': 0.5,
				},
			},
			'iMSN': {
				'Name': 'Indirect-pathway MSNs',
				'e'   : 0.2,
				'DA'  : -1,             # Sign modifier (iMSN DA is inhibitory)
				'W'   : {
					'Inp': 0.5,
					'Ctx': 0.5,
				},
			},
			'STN' : {
				'Name': 'Subthalamic nucleus',
				'e'   : -0.25,
				'W'   : {
					'Inp': 0.5,
					'Ctx': 0.5,
					'GPe': -1,
				},
			},
			'GPe' : {
				'Name': 'Globus pallidus (external)',
				'e'   : -0.2,
				'W'   : {
					'iMSN': -1,
					'STN' : 0.8,
				},
			},
			'SNr' : {
				'Name': 'Substantia nigra pars reticulata',
				'e'   : -0.2,
				'W'   : {
					'dMSN': -1,
					'STN' : 0.8,
					'GPe' : -0.4,
				},
			},
			'Thal': {
				'Name': 'Thalamus',
				'e'   : 0,
				'W'   : {
					'SNr': -1,
					'Ctx': 1,
					'TRN': {
						'between': -0.7,
						'within' : -0.1,
					}
				},
			},
			'TRN': {
				'Name': 'Thalamic reticular nucleus',
				'e'   : 0,
				'W'   : {
					'Ctx' : 1,
					'Thal': 1,
				}
			},
			'Ctx' : {
				'Name': 'Cortex',
				'e'   : 0,
				'W'   : {
					'Inp' : 1,
					'Thal': 1,
				},
			},
			'DA'  : {
				'Name': 'Dopamine output',
				'e'   : -0.2,
				'W'   : {},
			},
		}
