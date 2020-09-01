import sys
sys.path.append('..')
import brainsv2 as brains


class VisualCtx:
    def __init__( self ):
        self.image = None
    
    def updatePerception( self, im ):
        self.image = im
    
    def getReward( self ):
        self.perception

    def getSignal( self, hemi, color ):


class HypothalamusController:
    def __init__( self ):
        self.brain = brains.MotivationalBrain( Tb = 37.0, E = 1.0 )
        tempSensorL = TemperatureSensor( ego_angle = -np.pi/2.0, environment = self.visualCtx, 'red' )
		tempSensorR = TemperatureSensor( ego_angle = np.pi/2.0, environment = self.visualCtx, 'red' )
		chemSensorL = ChemicalSensor( ego_angle = -np.pi/2.0, environment = self.visualCtx, 'green' )
		chemSensorR = ChemicalSensor( ego_angle = np.pi/2.0, environment = self.visualCtx, 'green' )
        brain.registerSensor( 'Tl', tempSensorL )
		brain.registerSensor( 'Tr', tempSensorR )	
		brain.registerSensor( 'Fl', chemSensorL )
		brain.registerSensor( 'Fr', chemSensorR )

    def step( self, im, h, t ):
        self.visualCtx.updatePerception( im )
        F = self.environment.getReward()
		self.brain.step( h, t, F )
