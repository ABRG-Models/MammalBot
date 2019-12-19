import copy
import math
import numpy as np


class BasalGanglia(object):
    def __init__(self, channels: int):

        # # Set model options
        # self.opt = {
        #     'Loop': True,    # MCx to dorsal striatum loop?
        #     'STR' : False,   # Striatal intraconnectivity?
        #     'TRN' : False,   # Use TRN?
        #     'Th_d' : False,   # VLT to dorsal striatum connectivity?
        # }

        # Artificial parameters
        # Gain
        k = 25
        dt = 0.01
        self.decay_constant = math.exp(-k * dt)
        # Slope
        self.m = 1  # Humphries & Gurney (2002)

        # Define the canonical BG structure
        # Weights are defined as 'TO': {'W': {'FROM': <WEIGHT>}}
        bg_core = {
            # Principal BG populations
            'dMSN'   : {
                'Name': 'Direct-pathway MSNs',
                'e'   : 0.2,
                'DA'  : 1,  # INVENTED
                'W'   : {
                    'Inp': 0.5,  # Humphries & Gurney (2002)
                    'Ctx': 0.5,  # Humphries & Gurney (2002)
                },
            },
            'iMSN'   : {
                'Name': 'Indirect-pathway MSNs',
                'e'   : 0.2,
                'DA'  : -1,  # INVENTED
                'W'   : {
                    'Inp': 0.5,  # Humphries & Gurney (2002)
                    'Ctx': 0.5,  # Humphries & Gurney (2002)
                },
            },
            'STN'    : {
                'Name': 'Subthalamic nucleus',
                'e'   : -0.25,
                'W'   : {
                    'Inp': 0.5,  # Humphries & Gurney (2002)
                    'Ctx': 0.5,  # Humphries & Gurney (2002)
                    'GPe': -1,  # Gurney, Prescott, & Redgrave (2001)
                    'PPn': 1,  # INVENTED (Duplicate of DA projection)
                },
            },
            'GPe'    : {
                'Name': 'Globus pallidus (external)',
                'e'   : -0.2,
                'W'   : {
                    'iMSN': -1,  # Gurney, Prescott, & Redgrave (2001)
                    'STN' : 0.8,  # Humphries & Gurney (2002)
                },
            },
            'GPi_SNr': {
                'Name': 'Globus pallidus (internal) / Substantia nigra pars reticulata',
                'e'   : -0.2,
                'W'   : {
                    'dMSN': -1,  # Gurney, Prescott, & Redgrave (2001)
                    'STN' : 0.8,  # Humphries & Gurney (2002)
                    'GPe' : -0.4,  # Humphries & Gurney (2002)
                },
            },
            # Additional populations
            'Ctx'    : {
                'Name': 'Cortex',
                'e'   : 0,
                'W'   : {
                    # 'Inp' : 1,        # Humphries & Gurney (2002)
                    # 'Thal': 1,        # Humphries & Gurney (2002)
                    'Inp' : 0.5,
                    'Thal': 0.5,
                },
            },
            'Thal'   : {
                'Name': 'Thalamus',
                'e'   : 0,
                'W'   : {
                    'GPi_SNr': -1,  # Humphries & Gurney (2002)
                    'Ctx'    : 1,  # Humphries & Gurney (2002)
                },
            },
            # Justification for creating ventral and dorsal PPn populations:
            # "There is evidence that anterior cholinergic PPn neurons preferentially project to the SNc,
            #  whereas posterior cholinergic PPn cells preferentially project to the VTA (Oakman et al., 1995)."
            # - Humphries and Prescott (2010), pp 392
            'PPn'    : {
                'Name': 'Pedunculopontine nucleus',
                'e'   : -0.15,
                'W'   : {
                    'GPe'    : -0.5,  # Humphries & Prescott (2010)
                    # 'GPe'    : -0.3
                    'STN'    : 0.8,  # Made up by me (copied from other STN projections)
                    'GPi_SNr': -0.5,  # Made up by me
                }
            },
            'VTA_SNc': {
                'Name': 'Ventral tegmental area / Substantia nigra pars compacta',
                'e'   : -0.075,  # Humphries & Prescott (2010)
                'W'   : {
                    'dMSN': -1,  # INVENTED
                    'iMSN': -1,  # INVENTED
                },
            },
            'DA'     : {
                'Name': 'Dopamine neurons',
                'e'   : -0.2,
                'W'   : {
                    'VTA_SNc': -1,  # INVENTED
                    'PPn'    : 1,  # Humphries & Prescott (2010) (to VTA)
                },
            },
        }

        # Create dorsal and ventral BG structures from core layout
        self.pop = {
            'Dorsal' : copy.deepcopy(bg_core),
            'Ventral': copy.deepcopy(bg_core),
        }

        # Define lateral hypothalamus
        self.pop['Ventral']['LH'] = {
            'Name': 'Lateral hypothalamus',
            'e'   : 0,
            'W'   : {},
        }

        # Region-specific modifications
        self.pop['Dorsal']['VTA_SNc']['W']['Ventral'] = {
            'dMSN': -1,  # TODO: Calibrate
            'iMSN': -1,  # TODO: Calibrate
        }

        # Lateral hypothalamus to ventral SNc weight
        self.pop['Ventral']['DA']['W']['LH'] = 1  # TODO: Calibrate

        # self.pop['Dorsal']['VTA_SNc']['W']['Ventral']['LH'] = 1  # TODO: Calibrate

        # Input, activity and output arrays
        self.input = np.zeros(channels)
        for region in self.pop.keys():
            for p in self.pop[region].keys():
                self.pop[region][p]['a'] = np.zeros(channels)
                self.pop[region][p]['o'] = np.zeros(channels)

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

        for src_name, src_weight in self.pop[dst_region][dst_pop]['W'].items():
            if src_name is not 'Inp':

                # Special case: Input from remote region
                if src_name in self.pop.keys():
                    src_region = src_name

                    for remote_name, remote_weight in self.pop[dst_region][dst_pop]['W'][src_region].items():
                        remote_output = self.pop[src_region][remote_name]['o']
                        u = u + remote_weight * remote_output

                # General case: Input from local region
                else:
                    src_region = dst_region
                    src_output = self.pop[src_region][src_name]['o']

                    # Special case: STN projections are not channel specific
                    if src_name == 'STN':
                        u = u + src_weight * sum(src_output)

                    # # Special case: TRN
                    # elif src_name == 'TRN':
                    #     u = u + (
                    #             src_weight['within'] * src_output
                    #             + src_weight['between'] * (sum(src_output) * np.ones(len(src_output)) - src_output)
                    #     )

                    # General case: Sum of weighted inputs
                    else:
                        u = u + src_weight * src_output

            # Special case: Model inputs
            else:
                u = u + src_weight * self.input

        # Dopamine modifier
        if 'DA' in self.pop[dst_region][dst_pop].keys():
            # print(dst_region + ' ' + dst_pop + ' DA modification:')
            # print('DA activation = ' + str(self.pop[dst_region]['DA']['o']) + ' * ' + str(self.pop[dst_region][dst_pop]['DA']))
            # print('u = ' + str(u))

            u = u * (1 + self.pop[dst_region]['DA']['o'] * self.pop[dst_region][dst_pop]['DA'])

            # print('Now u = ' + str(u))

        return u

    def step(self, c):
        self.input = c

        for region in self.pop.keys():
            for p in self.pop[region].keys():
                # Outputs
                self.pop[region][p]['o'] = self.ramp(self.pop[region][p]['a'], self.pop[region][p]['e'], self.m)

                # FIXED LH OUTPUT FOR TESTING ONLY
                self.pop['Ventral']['LH']['o'] = np.array([1, 1, 1, 0, 0, 0])

                # Recovery variables
                self.pop[region][p]['u'] = self.activation(region, p)

                # Leaky integrator activation
                # (Eq. 1 from Gurney, Prescott, & Redgrave (2001b))
                self.pop[region][p]['a'] = (self.pop[region][p]['a'] - self.pop[region][p]['u']) \
                                           * self.decay_constant + self.pop[region][p]['u']
