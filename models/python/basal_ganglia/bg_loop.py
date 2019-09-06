import math
import numpy as np


class BasalGanglia(object):
    def __init__(self, channels):

        # Use TRN?
        self.trn = True

        # Striatal intraconnectivity?
        self.str = 0

        # VLT to striatal connectivity?
        self.vlt = 0

        # MCx to striatal loop?
        self.loop = 1

        # Model parameters
        # Dopamine
        self.da = {
            'SEL': 0.2,
            'CON': 0.2
        }

        # Artificial parameters
        # Gain
        k = 25
        dt = 0.01
        self.decay_constant = math.exp(-k * dt)
        # Slope
        self.m = 1

        # Weights, defined as 'FROM': {'TO': W}
        self.W = {
            'Inp': {
                'SEL': 0.5,
                'CON': 0.5,
                'STN': 0.5,
                'MCx': 1,
            },
            'SEL': {
                'SEL': -0.2,
                'CON': -0.05,
                'GPi': -1,
            },
            'CON': {
                'SEL': -0.2,
                'CON': -0.2,
                'GPe': -1,
            },
            'STN': {
                'GPe': 0.8,
                'GPi': 0.8,
            },
            'GPe': {
                'STN': -1,
                'GPi': -0.4,
            },
            'GPi': {
                'VLT': -1,
            },
            'VLT': {
                'SEL': 0.4,
                'CON': 0.4,
                'TRN': 1,
                'MCx': 1,
            },
            'TRN': {
                'between': -0.7,
                'within' : -0.1,
            },
            'MCx': {
                'SEL': 0.5,
                'CON': 0.5,
                'STN': 0.5,
                'VLT': 1,
                'TRN': 1,
            }
        }

        # Modify weights to exclude populations
        if not self.trn:
            self.W['TRN']['between'] = 0
            self.W['TRN']['within'] = 0

        if not self.str:
            self.W['SEL']['SEL'] = 0
            self.W['SEL']['CON'] = 0
            self.W['CON']['SEL'] = 0
            self.W['CON']['CON'] = 0

        if not self.vlt:
            self.W['VLT']['SEL'] = 0
            self.W['VLT']['CON'] = 0

        if not self.loop:
            self.W['MCx']['SEL'] = 0
            self.W['MCx']['CON'] = 0
            self.W['MCx']['STN'] = 0

        # Populations with offset values
        self.pop = {
            'SEL': {
                'Name': 'Striatum D1',
                'e'   : 0.2
            },
            'CON': {
                'Name': 'Striatum D2',
                'e'   : 0.2
            },
            'STN': {
                'Name': 'Subthalamic nucleus',
                'e'   : -0.25
            },
            'GPe': {
                'Name': 'Globus pallidus (external)',
                'e'   : -0.2
            },
            'GPi': {
                'Name': 'Globus pallidus (internal)',
                'e'   : -0.2
            },
            'VLT': {
                'Name': 'Ventrolateral thalamus',
                'e'   : 0
            },
            'TRN': {
                'Name': 'Thalamic reticular nucleus',
                'e'   : 0
            },
            'MCx': {
                'Name': 'Motor cortex',
                'e'   : 0
            }
        }

        # Activity and output arrays
        for pop in self.pop.keys():
            self.pop[pop]['a'] = np.zeros(channels)
            self.pop[pop]['o'] = np.zeros(channels)

    @staticmethod
    def ramp(a, e, m):
        ramp_output = np.zeros(len(a))

        a_min = np.where(a < e)
        a_rmp = np.where((a >= e) & (a <= 1 / m + e))
        a_max = np.where(a > 1 / m + e)

        ramp_output[a_min] = 0
        ramp_output[a_rmp] = m * (a[a_rmp] - e)
        ramp_output[a_max] = 1

        return ramp_output

    def update(self, c):

        # Outputs
        for pop in self.pop.keys():
            self.pop[pop]['o'] = self.ramp(self.pop[pop]['a'], self.pop[pop]['e'], self.m)

        # Recovery variables
        # Striatum D1
        self.pop['SEL']['u'] = (
                (
                    c * self.W['Inp']['SEL']
                    + self.pop['MCx']['o'] * self.W['MCx']['SEL']
                    + sum(self.pop['SEL']['o']) * self.W['SEL']['SEL']
                    + sum(self.pop['CON']['o']) * self.W['CON']['SEL']
                    + self.pop['VLT']['o'] * self.W['VLT']['SEL']
                )
                * (1 + self.da['SEL'])
        )

        # Striatum D2
        self.pop['CON']['u'] = (
                (
                    c * self.W['Inp']['CON']
                    + self.pop['MCx']['o'] * self.W['MCx']['CON']
                    + sum(self.pop['SEL']['o']) * self.W['SEL']['CON']
                    + sum(self.pop['CON']['o']) * self.W['CON']['CON']
                    + self.pop['VLT']['o'] * self.W['VLT']['CON']
                )
                * (1 - self.da['CON'])
        )

        # STN
        self.pop['STN']['u'] = (
                c * self.W['Inp']['STN']
                + self.pop['GPe']['o'] * self.W['GPe']['STN']
        )

        # GPe
        self.pop['GPe']['u'] = (
                sum(self.pop['STN']['o']) * self.W['STN']['GPe']
                + self.pop['CON']['o'] * self.W['CON']['GPe']
        )

        # GPi
        self.pop['GPi']['u'] = (
                sum(self.pop['STN']['o']) * self.W['STN']['GPi']
                + self.pop['GPe']['o'] * self.W['GPe']['GPi']
                + self.pop['SEL']['o'] * self.W['SEL']['GPi']
        )

        # VL thalamus
        self.pop['VLT']['u'] = (
                self.pop['MCx']['o'] * self.W['MCx']['VLT']
                + self.pop['GPi']['o'] * self.W['GPi']['VLT']
                + self.pop['TRN']['o'] * self.W['TRN']['within']
                + (
                        (sum(self.pop['TRN']['o']) * np.ones(len(c))) - self.pop['TRN']['o']
                ) * self.W['TRN']['between']
        )

        # TRN
        self.pop['TRN']['u'] = (
                self.pop['MCx']['o'] * self.W['MCx']['TRN']
                + self.pop['VLT']['o'] * self.W['VLT']['TRN']
        )

        # Motor cortex
        self.pop['MCx']['u'] = (
                c * self.W['Inp']['MCx']
                + self.pop['VLT']['o'] * self.W['VLT']['MCx']
        )

        # Activation variables
        for pop in self.pop.keys():
            self.pop[pop]['a'] = (self.pop[pop]['a'] - self.pop[pop]['u']) * self.decay_constant + self.pop[pop]['u']
