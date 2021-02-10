
import dict_templates
import model_templates

from contextlib import suppress
from dataclasses import dataclass, InitVar, field
import math
import numpy as np
import sys

#
# @dataclass
# class SubClass:
# 	name: str
# 	values: dict = field(default_factory=dict)
#
#
# dict_template_infile = {
# 	'first_dict': SubClass(
# 		name='First entry',
# 		values={
# 			'One': 1,
# 			'Two': 2,
# 		},
# 	),
# }


@dataclass
class BasalGanglia:
	channels: int  # Number of input channels
	lh: InitVar[bool]  # Use lateral hypothalamus?
	trn: InitVar[bool]  # Use thalamic reticular nucleus?
	dict_field: dict = field(default_factory=dict)
	model: dict = field(default_factory=dict)

	# Artificial parameters
	# Gain
	k = 25
	dt = 0.01
	decay_constant = math.exp(-k * dt)
	# Slope
	m = 1  # Humphries & Gurney (2002)

	def __post_init__(self, lh, trn):
		self.dict_field['key_3'] = dict_templates.dict_template_import_one
		self.dict_field['key_4'] = dict_templates.dict_template_import_two
		self.dict_field['key_5'] = model_templates.HumphriesPrescott2010
		self.dict_field['key_6'] = model_templates.HumphriesGurney2002

		self.model['Ventral'] = model_templates.HumphriesPrescott2010
		self.model['Dorsal'] = model_templates.HumphriesGurney2002

		for reg in self.dict_field.keys():
			for pop in self.dict_field[reg].keys():
				# print(reg)
				# print(pop)
				pass

		# Input, activity and output arrays
		# self.input = np.zeros(self.channels)
		for reg in self.model.keys():
			for pop in self.model[reg].keys():
				self.model[reg][pop].a = np.zeros(self.channels)
				self.model[reg][pop].o = np.zeros(self.channels)


theclass = BasalGanglia(channels=3, lh=True, trn=False)

print(theclass)
