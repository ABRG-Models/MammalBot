#!/usr/bin/python
#
# Miro controller
# 

from PyQt5.QtCore import Qt, QTimer, QElapsedTimer
from basal_ganglia.bg_gurney import *
from hypothalamus.motivational_system import *
from model import *
import miro_functions.ball_approach_avoid.ball_detector as detector

class MiroController:

    def __init__(self):

        rospy.init_node("Miro_motivational", anonymous=True)

        self.views = []
        self.model = Model()
        self.robot = MiroClient()

        self.timer = QTimer()
        self.timer.timeout.connect(self.step)
    
        # wait for connect
        print "wait for connect..."
        time.sleep(1)

    def callback_package(self, msg):

        # ignore until active
        if not self.active:
            return

        # store
        self.package = msg

    def run( self ):
        self.timer.start(0)

    def step(self):

        # Step models
        self.model.step()
        # Update robot
        self.robot.step( self.model )

        # Notify views
        for v in self.views:
            v.notify( self.model )

        
    def add_view( self, view ):
        view.configure( self.model )
        self.views.append( view )


# Main run loop
if __name__ == '__main__':
    app = QApplication([])
    view = Window()
    control = MiroController()
    control.add_view( view )
    view.show()
    control.run()

    sys.exit(app.exec_())