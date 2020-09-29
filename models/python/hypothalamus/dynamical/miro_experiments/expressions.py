class Shine:
    def __init__( self, robot ):
        self.robot = robot
        
    def execute( self, color, xi ):
        idxs = []
        if color == 'red':
            idxs = range(int(xi*6))
            c = 0xFFFF0000
        elif color == 'green':
            idxs = [6 - x - 1 for x in range(int(xi*6))]
            c = 0xFF00FF00
        else:
            c = 0x00000000

        self.robot.shine( c, idxs )