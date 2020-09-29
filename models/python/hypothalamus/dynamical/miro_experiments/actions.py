import numpy as np
import time

class Action( object ):
    def __init__( self, robot, attention ):
        self.robot = robot
        self.attention = attention

    def start( self, listener ):
        pass

    def execute( self ):
        pass

    def cancel( self ):
        pass

class ActionOrient( Action ):
    def __init__( self, robot, attention ):
        super(ActionOrient, self).__init__(robot, attention)
        self.location = 0
        self._x = 0
        self.v = 0
        self.k = 0.1
        self.listener = None
        self.stop = True
        print "Initialixing or"

    def start( self, listener ):
        self.location = self.attention.x
        self.listener = listener
        self._x = 0
        self.v = 0

        if self.location == -1:
            self._x = -40.0
        elif self.location == 1:
            self._x = 40.0

        print "Starting Orienting: ", self.location, ", x: ", self._x

        self.stop = False
        self.robot.move_head( self._x )    

    def execute( self ):
        h = 0.1
        m = 0.1

        print " Executing Orienting: ", self.location, ", x: ", self._x

        if self._x > -1.0 and self._x < 1.0:
            self.listener.notifyStop()
            self._x = 0
            self.robot.move_head( self._x )
            self.stop = True

        if not self.stop:
            f = -self.k*self._x

            if self.location == -1:
                forces = [np.abs(f), 0]
            elif self.location == 1:
                forces = [0, np.abs(f)]
            else:
                forces = [0, 0]

            self.v += h*f/m
            self._x += h*self.v
            self.robot.move( forces )
            self.robot.move_head( self._x )

        def cancel( self ):
            self.stop = True
            self.robot.move( [0, 0] )


class ActionApproach( Action ):
    def __init__( self, robot, attention ):
        super(ActionApproach, self).__init__(robot, attention)
        self.stop = True
        self.F0 = 4.0
        self.t = 0.0

    def start( self, listener ):
        self.listener = listener
        self.stop = False
        self.f = self.F0
        self.t = 0.0

    def execute( self ):
        print "Executing approach: ", self.f
        if self.f < 0.1:
            self.listener.notifyStop()
            self.stop = True
            self.robot.move( [0, 0] )

        if not self.stop:
            forces = [self.f, self.f]
            self.robot.move( forces )

            self.f = self.F0*np.exp( -3*self.t )
            self.t += 0.05

    def cancel( self ):
        self.robot.move( [0, 0] )

class ActionAvoid( Action ):
    def __init__( self, robot, attention ):
        super(ActionAvoid, self).__init__(robot, attention)
        self.stop = True
        self.F0 = 3.0
        self.t = 0.0

    def start( self, listener ):
        self.listener = listener
        self.stop = False
        self.f = self.F0
        self.t = 0.0

    def execute( self ):
        if self.f < 0.1:
            self.listener.notifyStop()
            self.stop = True
            self.robot.move( [0, 0] )

        if not self.stop:
            forces = [-self.f, -self.f]
            self.robot.move( forces )

            self.f = self.F0*np.exp( -5*self.t )
            self.t += 0.05

    def cancel( self ):
        self.robot.move( [0, 0] )

class ActionSnap( Action ):
    def __init__( self, robot, attention ):
        super(ActionSnap, self).__init__(robot, attention)
        self.t = 0.0
        self.stop = True
        A = 0.3
        w = 2*np.pi*1.5
        self.f = lambda t: 0.5 - A*np.sin(w*t)

    def start( self, listener ):
        self.listener = listener
        self.stop = False
        self.t = 0.0

    def execute( self ): 
        if self.t > 1.0/1.5:
            self.stop = True
            self.listener.notifyStop()
            self.robot.tail_wag( 0.5 )

        if not self.stop: 
            print self.f(self.t)
            self.robot.tail_wag( self.f(self.t) )
            self.t += 0.05

    def cancel( self ):
        self.robot.tail_wag( 0.0 )
        self.stop = True

class ActionNull( Action ):
    def __init__( self, robot, attention ):
        super(ActionNull, self).__init__(robot, attention)
        self.t = 0.0
        self.stop = True
        

    def start( self, listener ):
        self.listener = listener
        self.stop = False
        self.t = 0.0

    def execute( self ): 
        if self.t > 1.0:
            self.stop = True
            self.listener.notifyStop()

        if not self.stop: 
            self.t += 0.05

    def cancel( self ):
        self.stop = True