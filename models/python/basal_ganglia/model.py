
class Model:
    def __init__(self):
        
        # Set fixed values
        self.BG_CHANNELS: int = 2
        self.FIXED_INPUTS = True
        self.BG_INPUTS = {
            0: {
                'Onset'    : 0.5,
                # 'Size'     : 0.4,
                'Size'     : 0.6,
                'Offset'   : 2,
                # 'Transient': {
                #     'Onset' : 3,
                #     'Offset': 4,
                #     'Size'  : 0.2,
                # },
            },
            1: {
                'Onset' : 0.6,
                'Size'  : 0.6,
                'Offset': 2.1,
            },
        }

        # Initialise basal ganglia
        self.lh = MotivationalSystem()
        self.bg = BasalGanglia(BG_CHANNELS)
        self.inputs = np.zeros(BG_CHANNELS)

    def step( self ):
        self.inputs = self.lh.step()
        self.inputs = self.ctx.step()
        self.bg.step( self.inputs )

