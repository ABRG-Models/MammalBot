import sys
sys.path.append('..')
import brainsv2 as brains
from sensors import *
import cv2
import numpy as np
import matplotlib.pyplot as plt
import actions
import action_patterns as ap
import cognitive as cog
import time
import motivational_systems as mt

class SearchTestController:
    def __init__( self, robot ):
        self.visual = cog.VisualPerception()
        self.attention = cog.AttentionSystem( self.visual )
        self.actionPattern = ap.SearchActionPattern( robot, self.attention )
        self.actionPattern.start()

    def cancel( self ):
        self.actionPattern.cancel()

    def step( self, im, h, t ):
        self.visual.update( im )
        self.attention.update()    
        self.actionPattern.execute()

    def plots( self ):
        pass

class FollowTestController:
    def __init__( self, robot ):
        self.visual = cog.VisualPerception()
        self.attention = cog.AttentionSystem( self.visual )
        self.attention.inhibit()
        self.actionPattern = ap.FollowActionPattern( robot, self.attention )
        self.started = False

    def cancel( self ):
        print "Triggering cancel!!!!!!"
        self.actionPattern.cancel()

    def step( self, im, h, t ):
        self.visual.update( im )
        # self.attention.update() 
        stim_x, stim_y, object_size = self.visual.getStimulusPositionAndSize(im, 'green') 
        
        if not self.started:
            print "Calling start: ", self.started
            self.actionPattern.start()
            self.started = True

        if stim_x is not None:
            self.attention.switch( -stim_x )
            
            self.actionPattern.execute()
        else:
            self.actionPattern.switch(0)

    def plots( self ):
        pass

class GreenTestController:
    def __init__( self, robot ):
        self.visual = cog.VisualPerception()
        self.attention = cog.AttentionSystem( self.visual )
        self.attention.inhibit()
        self.mt = mt.RedMotivationalSystem( robot, self.visual, self.attention )

    def cancel( self ):
        print "Triggering cancel!!!!!!"
        self.mt.cancel()

    def step( self, im, h, t ):
        self.mt.perceive( im, 1.0 )
        self.mt.express( 1.0 )
        self.mt.behave( 1.0 )

    def plots( self ):
        pass