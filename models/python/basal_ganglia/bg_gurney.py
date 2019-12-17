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

        # Model parameters
        # Dopamine
        # self.da = {
        #     # Tonic
        #     'dMSN_d': 0.2,
        #     'iMSN_d': -0.2,
        #     # 'VTA'   : {
        #     #     # TODO: Find good values for these, currently totally made up
        #     #     'dMSN_v': 0.5,
        #     #     'iMSN_v': 0.5,
        #     # }
        # }

        # Artificial parameters
        # Gain
        k = 25
        dt = 0.01
        self.decay_constant = math.exp(-k * dt)
        # Slope
        self.m = 1              # Humphries & Gurney (2002)

        # Weights, defined as 'TO': {'FROM': W}
        # Not defining optional extras right now
        # self.W = {
        #     # Dorsal BG
        #     'dMSN_d': {
        #         'Inp': 0.5,     # Humphries & Gurney (2002)
        #         'Ctx_d': 0.5,   # Humphries & Gurney (2002)
        #         # 'Th_d': 0.4,
        #     },
        #     'iMSN_d': {
        #         'Inp': 0.5,     # Humphries & Gurney (2002)
        #         'Ctx_d': 0.5,   # Humphries & Gurney (2002)
        #         # 'Th_d': 0.4,
        #     },
        #     'STN_d'   : {
        #         'Inp': 0.5,     # Humphries & Gurney (2002)
        #         'Ctx_d': 0.5,   # Humphries & Gurney (2002)
        #         'GPe': -1,      # Gurney, Prescott, & Redgrave (2001)
        #     },
        #     'GPe'   : {
        #         'iMSN_d': -1,   # Gurney, Prescott, & Redgrave (2001)
        #         'STN_d' : 0.8,  # Humphries & Gurney (2002)
        #     },
        #     'GPi'   : {
        #         'dMSN_d': -1,   # Gurney, Prescott, & Redgrave (2001)
        #         'STN_d' : 0.8,  # Humphries & Gurney (2002)
        #         'GPe'   : -0.4, # Humphries & Gurney (2002)
        #     },
        #     # TODO: Verify SNc connectivity, distinct from VTA?
        #     # TODO: Only patch projects to SNc
        #     'SNc'   : {
        #         'dMSN_d': -1,
        #         'dMSN_v': -1,
        #         # 'PPn'   : 1,
        #         # 'VP': -0.5,
        #     },
        #     # # Ventral BG
        #     # 'dMSN_v': {
        #     #     # TODO: Add ventral cortex connection
        #     #     'Inp': 0.25,
        #     # },
        #     # 'iMSN_v': {
        #     #     # TODO: Add ventral cortex connection
        #     #     'Inp': 0.35,
        #     # },
        #     # 'VP'    : {
        #     #     'iMSN_v': -1,
        #     # },
        #     # 'PPn'   : {
        #     #     'VP': -0.5,
        #     # },
        #     # 'VTA'   : {
        #     #     'dMSN_v': -1,
        #     #     'PPn'   : 1,
        #     #     'VP'    : -0.5,
        #     # },
        #     # Ventral BG
        #     'dMSN_v': {
        #         'Inp': 0.5,
        #         'Ctx_v': 0.5,
        #     },
        #     'iMSN_v': {
        #         'Inp': 0.5,
        #         'Ctx_v': 0.5,
        #     },
        #     'STN_v': {
        #         'Inp'  : 0.5,
        #         'Ctx_v': 0.5,
        #         'VP'  : -1,
        #     },
        #     'VP'    : {
        #         'iMSN_v': -1,   # Humphries & Prescott (2010)
        #         'STN_v': 0.8,
        #     },
        #     'PPn'   : {
        #         'VP': -0.5,     # Humphries & Prescott (2010)
        #         'STN_v': 0.8,
        #         'SNr'  : -0.5,  # Made up by me
        #     },
        #     'SNr'   : {
        #         'dMSN_v': -1,
        #         'STN_v' : 0.8,
        #         'VP'   : -0.4,
        #     },
        #     'VTA'   : {
        #         'dMSN_v': -1,   # Humphries & Prescott (2010)
        #         'PPn'   : 1,    # Humphries & Prescott (2010)
        #         'VP'    : -0.5, # Humphries & Prescott (2010)
        #     },
        #     # Other populations
        #     'Th_d'   : {
        #         'GPi': -1,
        #         'Ctx_d': 1,
        #         'TRN': {
        #             'between': -0.7,
        #             'within' : -0.1,
        #         },
        #     },
        #     'TRN'   : {
        #         'Ctx_d': 1,
        #         'Th_d': 1,
        #     },
        #     'Ctx_d'   : {
        #         # 'Inp': 0.5,
        #         'Inp': 1,
        #         'Th_d': 1,
        #     },
        # }

        # # Modify weights to exclude populations
        # if not self.opt['TRN']:
        #     self.W['TRN']['between'] = 0
        #     self.W['TRN']['within'] = 0
        #
        # if not self.opt['STR']:
        #     self.W['dMSN-d']['dMSN-d'] = 0
        #     self.W['dMSN-d']['iMSN-d'] = 0
        #     self.W['iMSN-d']['dMSN-d'] = 0
        #     self.W['iMSN-d']['iMSN-d'] = 0
        #
        # if not self.opt['Th_d']:
        #     self.W['Th_d']['dMSN-d'] = 0
        #     self.W['Th_d']['iMSN-d'] = 0
        #
        # if not self.opt['Loop']:
        #     self.W['Ctx_d']['dMSN-d'] = 0
        #     self.W['Ctx_d']['iMSN-d'] = 0
        #     self.W['Ctx_d']['STN_d'] = 0

        # TODO: Finish adding ventral BG to loop model
        # TODO: Integrate dopamine function
        # TODO: Unify pallidum / pallidus naming convention

        # # Populations with offset values
        # self.pop = {
        #     # Dorsal populations from Humphries & Gurney (2002)
        #     'dMSN_d': {
        #         'Name': 'Direct-pathway MSNs (dorsal)',
        #         'e'   : 0.2
        #     },
        #     'iMSN_d': {
        #         'Name': 'Indirect-pathway MSNs (dorsal)',
        #         'e'   : 0.2
        #     },
        #     'STN_d': {
        #         'Name': 'Subthalamic nucleus',
        #         'e'   : -0.25
        #     },
        #     'GPe': {
        #         'Name': 'Globus pallidus (external)',
        #         'e'   : -0.2
        #     },
        #     'GPi': {
        #         'Name': 'Globus pallidus (internal)',
        #         'e'   : -0.2
        #     },
        #     # SNc invented by me
        #     'SNc': {
        #         'Name': 'Substantia nigra pars compacta',
        #         # TODO: Check / verify SNc offset value (curently matching VTA)
        #         'e'   : -0.075
        #     },
        #     # Ventral populations from Humphries & Prescott (2010)
        #     #  https://www.sciencedirect.com/science/article/pii/S030100820900183X#app3
        #     'dMSN_v': {
        #         'Name': 'Direct-pathway MSNs (ventral)',
        #         'e'   : 0.2
        #     },
        #     'iMSN_v': {
        #         'Name': 'Indirect-pathway MSNs (ventral)',
        #         'e'   : 0.2
        #     },
        #     'PPn': {
        #         'Name': 'Pedunculopontine nucleus',
        #         'e'   : -0.15
        #     },
        #     'VP': {
        #         'Name': 'Ventral pallidum',
        #         'e'   : -0.2
        #     },
        #     'VTA': {
        #         'Name': 'Ventral tegmental area',
        #         'e'   : -0.075
        #     },
        #     # Non-BG populations
        #     'Ctx_d': {
        #         'Name': 'Motor cortex',
        #         'e'   : 0
        #     },
        #     'TRN': {
        #         'Name': 'Thalamic reticular nucleus',
        #         'e'   : 0
        #     },
        #     'Th_d': {
        #         'Name': 'Ventrolateral thalamus',
        #         'e'   : 0
        #     },
        # }

        # Define the canonical BG structure
        bg_core = {
            # Principal BG populations
            'dMSN': {
                'Name': 'Direct-pathway MSNs',
                'e'   : 0.2,
                'DA'  : 0.8,  # INVENTED
                'W'   : {
                    # TODO: Add DA modulation
                    # 'Inp': 0.5,     # Humphries & Gurney (2002)
                    # 'Ctx': 0.5,     # Humphries & Gurney (2002)
                    'Inp': 0.3,
                    'Ctx': 0.3,
                    # 'DA' : 0.8,       # INVENTED
                },
            },
            'iMSN': {
                'Name': 'Indirect-pathway MSNs',
                'e'   : 0.2,
                'DA'  : -0.8,  # INVENTED
                'W'   : {
                    # TODO: Add DA modulation (or done separately?)
                    # 'Inp': 0.5,     # Humphries & Gurney (2002)
                    # 'Ctx': 0.5,     # Humphries & Gurney (2002)
                    'Inp': 0.3,
                    'Ctx': 0.3,
                    # 'DA' : -0.8,       # INVENTED
                },
            },
            'STN' : {
                'Name': 'Subthalamic nucleus',
                'e'   : -0.25,
                'W'   : {
                    'Inp': 0.5,     # Humphries & Gurney (2002)
                    'Ctx': 0.5,     # Humphries & Gurney (2002)
                    'GPe': -1,      # Gurney, Prescott, & Redgrave (2001)
                    'PPn': 1,       # INVENTED (Duplicate of DA projection)
                },
            },
            'GPe' : {
                'Name': 'Globus pallidus (external)',
                'e'   : -0.2,
                'W'   : {
                    'iMSN': -1,     # Gurney, Prescott, & Redgrave (2001)
                    'STN' : 0.8,    # Humphries & Gurney (2002)
                },
            },
            'GPi_SNr' : {
                'Name': 'Globus pallidus (internal) / Substantia nigra pars reticulata',
                'e'   : -0.2,
                'W'   : {
                    'dMSN': -1,     # Gurney, Prescott, & Redgrave (2001)
                    'STN' : 0.8,    # Humphries & Gurney (2002)
                    'GPe' : -0.4,   # Humphries & Gurney (2002)
                },
            },
            # Additional populations
            'Ctx' : {
                'Name': 'Cortex',
                'e'   : 0,
                'W'   : {
                    # 'Inp' : 1,      # Humphries & Gurney (2002)
                    # 'Thal': 1,      # Humphries & Gurney (2002)
                    'Inp' : 0.5,
                    'Thal': 0.5,
                },
            },
            'Thal': {
                'Name': 'Thalamus',
                'e'   : 0,
                'W'   : {
                    'GPi_SNr': -1,      # Humphries & Gurney (2002)
                    'Ctx': 1,       # Humphries & Gurney (2002)
                },
            },
            # "There is evidence for separate cholinergic PPn populations projecting to SNc and VTA"
            # Humphries and Prescott (2010), pp 389
            'PPn': {
                'Name': 'Pedunculopontine nucleus',
                'e'   : -0.15,
                'W'   : {
                    'GPe'    : -0.5,    # Humphries & Prescott (2010)
                    'STN'    : 0.8,     # Made up by me (copied from other STN projections)
                    'GPi_SNr': -0.5,    # Made up by me
                }
            },
            'VTA_SNc'  : {
                'Name': 'Ventral tegmental area / Substantia nigra pars compacta',
                'e'   : -0.075,         # Humphries & Prescott (2010)
                'W'   : {
                    'dMSN': -1,         # INVENTED
                    'iMSN': -1,         # INVENTED
                },
            },
            'DA'    : {
                'Name': 'Dopamine neurons',
                'e'   : -0.2,
                'W'   : {
                    'VTA_SNc': -1,      # INVENTED
                    'PPn'    : 1,       # Humphries & Prescott (2010) (to VTA)
                },
            },
        }

        # Create dorsal and ventral BG structures
        self.pop = {
            'Dorsal' : copy.deepcopy(bg_core),
            'Ventral': copy.deepcopy(bg_core),
        }

        # Region-specific modifications
        self.pop['Dorsal']['VTA_SNc']['W']['Ventral'] = {
            'dMSN': -1,  # TODO: Calibrate
            'iMSN': -1,  # TODO: Calibrate
        }

        # Lateral hypothalamus to ventral SNc weight
        self.pop['Ventral']['VTA_SNc']['W']['LH'] = 0    # TODO: Calibrate

        # # PPn to DA weight
        # self.pop['Ventral']['DA']['W']['PPn'] = 1

        # Define lateral hypothalamus
        self.pop['Ventral']['LH'] = {
            'Name': 'Lateral hypothalamus',
            'e'   : 0,
            'W'   : {},
        }

        # # PPn
        # self.pop['Ventral']['PPn'] = {
        #     'Name': 'Pendunculopontine nucleus',
        #     'e'   : -0.15,      # Humphries & Prescott (2010)
        #     'W'   : {
        #         'GPe': -0.5     # Humphries & Prescott (2010)
        #     }
        # }

        self.pop['Ventral']['PPn'] = {
            'Name': 'Pedunculopontine nucleus',
            'e'   : -0.15,
            'W'   : {
                'GPe': -0.5,        # Humphries & Prescott (2010)
                'STN': 0.8,
                'GPi_SNr': -0.5,        # Made up by me
            }
        }

        # TODO: Modify structures to match dorsal / ventral anatomy and connections

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
                # TODO: Connections from external regions need testing
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

                    # Special case: STN
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
            print(dst_region + ' ' + dst_pop + ' DA modification:')
            print('DA activation = ' + str(self.pop[dst_region]['DA']['o']) + ' * ' + str(self.pop[dst_region][dst_pop]['DA']))
            print('u = ' + str(u))

            u = u * (1 + self.pop[dst_region]['DA']['o'] * self.pop[dst_region][dst_pop]['DA'])

            print('Now u = ' + str(u))

        return u

    def step(self, c):
        # Input
        self.input = c

        # TODO: Update all step routines to operate across all BG substructures

        # Outputs
        for region in self.pop.keys():
            for p in self.pop[region].keys():
                self.pop[region][p]['o'] = self.ramp(self.pop[region][p]['a'], self.pop[region][p]['e'], self.m)

        # FIXED LH OUTPUT FOR TESTING
        self.pop['Ventral']['LH']['o'] = np.array([1, 1, 1, 0, 0, 0])

        # Recovery variables
        for region in self.pop.keys():
            for p in self.pop[region].keys():
                self.pop[region][p]['u'] = self.activation(region, p)

        # Leaky integrator activation
        for region in self.pop.keys():
            for p in self.pop[region].keys():
                # Eq. 1 from Gurney, Prescott, & Redgrave (2001b)
                self.pop[region][p]['a'] = (self.pop[region][p]['a'] - self.pop[region][p]['u']) \
                                           * self.decay_constant + self.pop[region][p]['u']
