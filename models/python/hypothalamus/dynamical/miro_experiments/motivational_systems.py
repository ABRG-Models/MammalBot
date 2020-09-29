import action_patterns as ap
import cognitive as cog
import expressions as exp

class MotivationalSystem(object):
    def __init__(self, robot, perception, attention):
        self.robot = robot
        self.action_patterns = []
        self.perception = perception
        self.attention = attention
        self.object_size = 0
        self.stim_x = None
        self.c_ap = -1
        self.start = True
        self.shine = exp.Shine( robot )
        self.count = 0

    def perceive(self, images, xi):
        pass

    def express( self, xi ):
        pass

    def behave( self, xi ):
        pass

    def cancel( self ):
        self.attention.obstacle = True
        self.action_patterns[self.c_ap].cancel()

    def start_pattern( self, idx ):
        if self.c_ap == idx:
            self.start = False
        else:
            self.cancel()
            self.start = True

        self.c_ap = idx

    def iterate( self, images, xi ):
        self.perceive( images, xi )
        self.express( xi )
        self.behave( xi )

        r = self.perception.isClose( images, self.object_size )
        self.attention.obstacle = False
        return r

class RedMotivationalSystem(MotivationalSystem):
    def __init__(self, robot, perception, attention):
        super(RedMotivationalSystem, self).__init__( robot, perception, attention )
        self.action_patterns = [ap.SearchActionPattern( self.robot, self.attention ), 
                                ap.FollowActionPattern(self.robot, self.attention),
                                ap.ConsumeActionPattern(self.robot, self.attention)]

    def perceive(self, images, xi):
        if images is None:
            stim_x = None
            stim_y = None
            return 

        self.perception.update( images )
        self.attention.update( )

        if self.count < 5: 
            stim_x, stim_y, object_size = self.perception.getStimulusPositionAndSize(images, 'red')
            
            if stim_x is not None:
                self.stim_x = stim_x
                self.object_size = object_size

            self.count += 1
        else:

            if self.stim_x is None:
                self.start_pattern( 0 )
            elif self.perception.isClose( images, self.object_size ) :
                self.start_pattern( 2 )
            else:
                self.attention.switch( -self.stim_x )
                self.start_pattern( 1 )

            self.count = 0
            self.stim_x = None
        
    def express( self, xi ):
        self.shine.execute( 'red', xi )

    def behave( self, xi ):
        if self.start:
            self.action_patterns[self.c_ap].start()

        self.action_patterns[self.c_ap].execute()
        

class GreenMotivationalSystem(MotivationalSystem):
    def __init__(self, robot, perception, attention):
        super(GreenMotivationalSystem, self).__init__( robot, perception, attention )
        self.action_patterns = [ap.SearchActionPattern( self.robot, self.attention ), 
                                ap.FollowActionPattern(self.robot, self.attention),
                                ap.ConsumeActionPattern(self.robot, self.attention)]

    def perceive(self, images, xi):
        if images is None:
            stim_x = None
            stim_y = None
            return 

        self.perception.update( images )
        self.attention.update( )
        stim_x, stim_y, self.object_size = self.perception.getStimulusPositionAndSize(images, 'green')

        if stim_x is None:
            self.start_pattern( 0 )
        elif self.perception.isClose( images, self.object_size ):
            self.start_pattern( 2 )
        else:
            self.attention.switch( -stim_x )
            self.start_pattern( 1 )
        
    def express( self, xi ):
        self.shine.execute( 'green', xi )

    def behave( self, xi ):
        if self.start:
            self.action_patterns[self.c_ap].start()

        self.action_patterns[self.c_ap].execute()
