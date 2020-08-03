# Recreation of model from Mannella, Mirolli, & Baldassarre (2016)
import numpy as np


class BasalGanglia(object):
    def __init__(self, channels):

        # Parameters of neural units
        # Table A2
        parameters = {
            'BG'   : {
                'tau'  : 300,
                'sigma': 1,
                'theta': 0,
            },
            'Th'   : {
                'tau'  : 300,
                'sigma': 1,
                'theta': 0,
            },
            'Ctx'  : {
                'tau'  : 2000,
                'sigma': 20,
                'theta': 0.8,
            },
            'DA'   : {
                'tau'  : 300,
                'sigma': 1,
                'theta': 1,
            },
            'Onset': {
                'sigma': 1,
                'theta': 0,
                'tau_o': 100,
                'tau_i': 500,
            },
            'Noise': {
                'tau_noise': 80,
            }
        }

        # Neural populations
        # Include data from Table A4
        self.pop = {
            # Basal ganglia
            'DLS': {
                'Name'  : 'Dorsolateral striatum',
                'Type'  : 'Striatum',
                'Region': 'BG',
                **parameters['BG'],
                'iota'     : 0.2,
                'delta'    : 4,
                'eta'      : 0.02,
                'theta_da' : 0.8,
                'theta_DLS': 0.5,
                'theta_inp': 0.5,
                'max_w'    : 1,
                'da_input' : 'SNco',
            },
            'DMS': {
                'Name'  : 'Dorsomedial striatum',
                'Type'  : 'Striatum',
                'Region': 'BG',
                **parameters['BG'],
                'iota'     : 0.5,
                'delta'    : 6.5,
                'eta'      : 0.02,
                'theta_da' : 0.8,
                'theta_DMS': 0.5,
                'theta_inp': 0.5,
                'max_w'    : 1,
                'da_input' : 'SNco',
            },
            'NAc': {
                'Name'  : 'Nucleus accumbens (core)',
                'Type'  : 'Striatum',
                'Region': 'BG',
                **parameters['BG'],
                'iota'     : 0.8,
                'delta'    : 1.5,
                'eta'      : 0.05,
                'theta_da' : 0.9,
                'theta_NAc': 0.9,
                'theta_inp': 0.9,
                'max_w'    : 2,
                'da_input' : 'VTA',

            },
            'STNdl': {
                'Name'  : 'Dorsolateral subthalamic nucleus',
                'Type'  : 'Leaky',
                'Region': 'BG',
                **parameters['BG'],
            },
            'STNdm': {
                'Name'  : 'Dorsomedial subthalamic nucleus',
                'Type'  : 'Leaky',
                'Region': 'BG',
                **parameters['BG'],
            },
            'STNv': {
                'Name'  : 'Ventral subthalamic nucleus',
                'Type'  : 'Leaky',
                'Region': 'BG',
                **parameters['BG'],
            },
            'GPi': {
                'Name'  : 'Globus pallidus (internal)',
                'Type'  : 'Leaky',
                'Region': 'BG',
                **parameters['BG'],
            },
            'GPi/SNr': {
                'Name'  : 'Globus pallidus (internal) / Substantia nigra pars reticulata',
                'Type'  : 'Leaky',
                'Region': 'BG',
                **parameters['BG'],
            },
            'SNr': {
                'Name'  : 'Substantia nigra pars reticulata',
                'Type'  : 'Leaky',
                'Region': 'BG',
                **parameters['BG'],
            },
            # Thalamus
            'MGV': {
                'Name':   'Thalamus medial geniculate body, ventral division',
                'Type'  : 'Leaky',
                'Region': 'Th',
                **parameters['Th'],
                **parameters['Noise'],
                'upsilon': 0.25,
            },
            'P': {
                'Name'  : 'Pulvinar, part of thalamus',
                'Type'  : 'Leaky',
                'Region': 'Th',
                **parameters['Th'],
                **parameters['Noise'],
                'upsilon': 0.25,
            },
            'DM': {
                'Name'  : 'Dorsomedial thalamus',
                'Type'  : 'Leaky',
                'Region': 'Th',
                **parameters['Th'],
                **parameters['Noise'],
                'upsilon': 6,
            },
            # Cortex
            'MC': {
                'Name'  : 'Motor cortex',
                'Type'  : 'Leaky',
                'Region': 'Ctx',
                **parameters['Ctx'],
                'theta_MC': 0.8
            },
            'PFCd/PC': {
                'Name'  : 'Prefrontal cortex, dorsal division',
                'Type'  : 'Leaky',
                'Region': 'Ctx',
                **parameters['Ctx'],
            },
            'PL': {
                'Name'  : 'Prelimbic cortex',
                'Type'  : 'Leaky',
                'Region': 'Ctx',
                **parameters['Ctx'],
            },
            # Dopamine
            'SNco': {
                'Name'  : 'Substantia nigra pars compacta (O)',
                'Type'  : 'Leaky',
                'Region': 'DA',
                **parameters['DA'],
            },
            'SNci': {
                'Name'  : 'Substantia nigra pars compacta (I)',
                'Type'  : 'Leaky',
                'Region': 'DA',
                **parameters['DA'],
            },
            'VTA': {
                'Name'  : 'Ventral tegmental area',
                'Type'  : 'Leaky',
                'Region': 'DA',
                **parameters['DA'],
            },
            # Onset units
            'PPN': {
                'Name'  : 'Peduncolopontine nucleus',
                'Type'  : 'Onset',
                'Region': 'Onset',
                **parameters['Onset'],
            },
            'LH': {
                'Name'  : 'Lateral hypothalamus',
                'Type'  : 'Onset',
                'Region': 'Onset',
                **parameters['Onset'],
            },
            'BLA': {
                'Name'  : 'Amygdala, basolateral complex',
                'Type'  : 'Onset',
                'Region': 'Onset',
                **parameters['Onset'],
                'tau_o': 500,
                'tau_tra': 500,
                'alpha': 10**10,
                'eta': 0.08,
                'theta_da': 0.7
            },
            # # Faux units added just to make this work for now
            # 'Mani': {
            #     'Name': 'Manipulanda',
            #     'Type': 'None'
            # },
            # 'Food': {
            #     'Name': 'Food',
            #     'Type': 'None',
            # },
            'Input': {
                'Name'  : 'Model input',
                'Type'  : 'None',
                'Region': 'None',
            }
        }

        # Connection weights, defined as 'TO': {'FROM': W}
        # Table A3
        self.W = {
            # Dorsolateral BG
            'DLS': {
                # 'Mani': 0,
                'Input': 1,
                'MC'   : 1,
            },
            'STNdl': {
                'MC': 1.6,
            },
            'GPi': {
                'DLS'  : -3,
                'STNdl': -2,
            },
            # Dorsomedial BG
            'DMS': {
                # 'Mani'   : 0,
                'Input'  : 1,
                'PFCd/PC': 1,
            },
            'STNdm': {
                'PFCd/PC': 1.6,
            },
            'GPi/SNr': {
                'DMS'  : -3,
                'STNdm': -2,
            },
            # Ventral BG
            'NAc': {
                # 'BLA': 0,
                'Input': 1,
                'PL'   : 1,
            },
            # In paper STNv is listed as STNdm, presumably a typo
            'STNv': {
                'PL': 1.6,
            },
            'SNr': {
                'NAc' : -3,
                'STNv': -2,
            },
            # Thalamus
            # In paper all 'from' populations are listed as 'GPi/SNr'
            'MGV': {
                'MGV': -0.8,
                'GPi': 1.5,
            },
            'P': {
                'P'      : -0.8,
                'GPi/SNr': 1.5,
            },
            'DM': {
                'DM' : -0.8,
                'SNr': 1.5,
            },
            # Cortex
            # In paper all 'from' populations are listed as 'Th'
            'MC': {
                'PFCd/PC': 1,
                'MGV'    : 1,
            },
            'PFCd/PC': {
                'MC': 0.2,
                'PL': 0.2,
                'P' : 1,
            },
            'PL': {
                'PFCd/PC': 1,
                'DM'     : 1,
            },
            # Dopamine
            'SNco': {
                'SNci': 1,
                'PPN' : 20,
            },
            'SNci': {
                'DMS': -10,
                'NAc': -6,
            },
            'VTA': {
                'LH': 20,
            },
            # Onset units
            'BLA': {
                # 'Mani': 5,
                # 'Food': 5,
                # 'Sat' : 10,
                'BLA' : 0,
            },
            'PPN': {
                # 'Food': 10,
            },
            'LH': {
                # 'Food': 10,
                'BLA' : 5,
            },
        }

        # Activity and output arrays
        for p in self.pop.keys():
            if self.pop[p]['Type'] == 'Leaky' or self.pop[p]['Type'] == 'Striatum':
                # Activation potentials
                self.pop[p]['I'] = np.zeros(channels)
                self.pop[p]['u'] = np.zeros(channels)
                # Activation
                self.pop[p]['v'] = np.zeros(channels)

                # Thalamic noise
                if self.pop[p]['Region'] == 'Th':
                    self.pop[p]['n'] = np.zeros(channels)

            elif self.pop[p]['Type'] == 'Onset':
                # Activation potentials
                self.pop[p]['u_o'] = np.zeros(channels)
                self.pop[p]['u_i'] = np.zeros(channels)
                # Activation
                # self.pop[p]['o'] = np.zeros(channels)
                self.pop[p]['v'] = np.zeros(channels)

            elif self.pop[p]['Type'] == 'None':
                pass

            else:
                print('Error: Invalid neural type while setting up arrays')

    def step(self, c):
        # # Temp just to make this work
        # self.pop['Mani']['v'] = mani
        # self.pop['Food']['v'] = food

        self.pop['Input']['v'] = c

        # Activation potential
        for p in self.pop.keys():
            # print('Population: ' + self.pop[p]['Name'])
            # Onset populations
            if self.pop[p]['Type'] == 'Onset':
                # Added because it needs to be here I think?
                self.pop[p]['I'] = self.signal_sum(p)

                # Activation potential
                # Eq. 4
                self.pop[p]['u_o'] = (-self.pop[p]['u_o'] + np.clip(self.pop[p]['I'] - self.pop[p]['u_i'], a_min=0, a_max=None)) / self.pop[p]['tau_o']

                self.pop[p]['u_i'] = (-self.pop[p]['u_i'] + self.pop[p]['I']) / self.pop[p]['tau_i']

                # Activation
                # self.pop[p]['o'] = max(0, np.tanh(self.pop[p]['u_o']))
                # Making this 'v' so it matches regular neuron activation
                # Eq. 5
                self.pop[p]['v'] = np.clip(np.tanh(self.pop[p]['u_o']), a_min=0, a_max=None)

            # TODO: Make ordering onset > other pops for all processes
            elif self.pop[p]['Type'] == 'Leaky':
                # Activation potential
                self.pop[p]['I'] = self.signal_sum(p)

                # # Thalamic noise
                # if self.pop[p]['Region'] == 'Th':
                #     n
                #     # self.pop[p]['I'] = self.pop[p]['I'] +

                self.pop[p]['u'] = self.activation_potential(
                    self.pop[p]['u'],
                    self.pop[p]['I'],
                    self.pop[p]['tau'],
                )

                # if p == 'SNr':
                #     print(self.pop[p]['u'])

                # Activation
                self.pop[p]['v'] = self.activation(self.pop[p]['sigma'], self.pop[p]['u'], self.pop[p]['theta'])

            elif self.pop[p]['Type'] == 'Striatum':
                # Activation potential
                self.pop[p]['I'] = self.signal_sum(p)

                self.pop[p]['u'] = self.activation_potential(
                    self.pop[p]['u'],
                    self.pop[p]['I'],
                    self.pop[p]['tau'],
                    iota=self.pop[p]['iota'],
                    delta=self.pop[p]['delta'],
                    # TODO: DA activation needs fixing
                    da=self.pop[self.pop[p]['da_input']]['v'],
                )

                # Activation
                self.pop[p]['v'] = self.activation(self.pop[p]['sigma'], self.pop[p]['u'], self.pop[p]['theta'])

            elif self.pop[p]['Type'] == 'None':
                pass

            else:
                print('Error: Invalid neural type while computing activation')

        # Testing out some changes
        # print(self.pop['NAc']['v'])
        self.pop['LH']['v'] = c
        # print(self.pop['VTA']['v'])

    def signal_sum(self, unit):
        I = 0
        # Eq. 1
        for input_pop in self.W[unit].items():
            name, w = input_pop
            I = I + (w * self.pop[name]['v'])

        # Thalamic noise
        if self.pop[unit]['Region'] == 'Th':
            self.pop[unit]['n'] = (
                    -self.pop[unit]['n']
                    + self.pop[unit]['upsilon']
                    * np.random.uniform(-0.5, 0.5, self.pop[unit]['n'].shape[0])
            ) / self.pop[unit]['tau_noise']

            I = I + self.pop[unit]['n']

            if unit == 'DM':
                print(self.pop[unit]['n'])

        return I

    @staticmethod
    def activation_potential(u, I, tau, **kwargs):
        if 'iota' in kwargs:
            # Eq. 3
            return (-u + (kwargs['iota'] + kwargs['delta'] * kwargs['da']) * I) / tau
        else:
            # Eq. 1
            return (-u + I) / tau

    @staticmethod
    def activation(sigma, u, theta):
        # Eq. 2
        return np.clip(np.tanh(sigma * (u - theta)), a_min=0, a_max=None)
