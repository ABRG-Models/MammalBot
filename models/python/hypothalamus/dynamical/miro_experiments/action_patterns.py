import actions
import numpy as np

class ActionPattern( object ):
    def __init__( self, robot, attention ):
        self.currentAction = None
        self.attention = attention
        self.actions = []

    def start( self ):
        pass

    def execute( self ):
        pass

    def switch( self, idx = -1 ):
        pass

    def notifyStop( self ):
        pass

    def cancel( self ):
        pass

class SearchActionPattern( ActionPattern ):
    def __init__( self, robot, attention ):
        super(SearchActionPattern, self).__init__(robot, attention )
        self.actions = [ actions.ActionOrient(robot, attention), 
                         actions.ActionNull( robot, attention ),
                         actions.ActionApproach( robot, attention ), 
                         actions.ActionAvoid( robot, attention ) ]
        self.c_id = 1

    def start( self ):
        self.attention.switch(-1 if np.random.rand() > 0.5 else 1)
        self.c_id = 1
        self.currentAction = self.actions[self.c_id]
        self.currentAction.start( self )

    def execute( self ):
        if self.currentAction is not None:
            self.currentAction.execute()

    def switch( self, idx = -1 ):
        
        if idx == -1:
            if self.c_id == 0:
                self.c_id = 1
            elif self.c_id == 1:
                self.c_id = 2
            else:
                self.attention.switch(-1 if np.random.rand() > 0.5 else 1)
                self.c_id = 0
        else:
            self.c_id = idx

        self.currentAction = self.actions[self.c_id]
        self.currentAction.start( self )

    def notifyStop( self ):
        self.switch()

    def cancel( self ):
        self.currentAction.cancel()
        self.switch( 3 )
    

class FollowActionPattern( ActionPattern ):
    def __init__( self, robot, attention ):
        super(FollowActionPattern, self).__init__(robot, attention )
        self.actions = [ actions.ActionOrient(robot, attention), actions.ActionNull( robot, attention ), actions.ActionApproach( robot, attention ), actions.ActionNull( robot, attention ) ]
        self.c_id = 0

    def start( self ):
        self.c_id = 0
        self.currentAction = self.actions[self.c_id]
        self.currentAction.start( self )

    def execute( self ):
        self.currentAction.execute()

    def switch( self, idx = -1 ):
        
        if idx == -1:
            if self.c_id == 0:
                self.c_id = 1
            elif self.c_id == 1:
                self.c_id = 2
            else:
                self.c_id = 0
        else:
            self.c_id = idx

        self.currentAction = self.actions[self.c_id]
        self.currentAction.start( self )

    def notifyStop( self ):
        self.switch()

    def cancel( self ):
        self.currentAction.cancel()
        self.switch( 2 )

class ConsumeActionPattern(ActionPattern):
    def __init__( self, robot, attention ):
        super(ConsumeActionPattern, self).__init__(robot, attention )
        self.actions = [ actions.ActionSnap(robot, attention) ]
        self.c_id = 0

    def start( self ):
        self.c_id = 0
        self.currentAction = self.actions[self.c_id]
        self.currentAction.start( self )

    def execute( self ):
        self.currentAction.execute()

    def switch( self, idx = -1 ):
        self.currentAction = self.actions[self.c_id]
        self.currentAction.start( self )

    def notifyStop( self ):
        self.switch()

    def cancel( self ):
        self.currentAction.cancel()
