import pyqtgraph as pg
import pyqtgraph.widgets.RemoteGraphicsView

import sys, os
from PyQt5.QtCore import Qt, QThreadPool, QRectF, pyqtSlot, QMutex, QPoint
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QHBoxLayout, QVBoxLayout, QToolBar, QWidget, QAction, QStatusBar, QOpenGLWidget
from PyQt5.QtGui import QPen, QBrush, QColor, QPixmap, QPainter, QSurfaceFormat, QLinearGradient, QIcon, QLineEdit, QPainterPath, QComboBox, QRegion

from simulation import *
import time 
import datetime

pg.setConfigOption( 'background', 'w' )
pg.setConfigOption( 'foreground', 'k' )

class MainWindow(QMainWindow):

	def __init__(self, *args, **kwargs):
		super(MainWindow, self).__init__(*args, **kwargs)

		self.setWindowTitle("Simulation of a random walk")
		self.setFixedSize( 900, 600 )

		self.threadpool = QThreadPool()

		# layout
		main_layout = QHBoxLayout()
		left_layout = QVBoxLayout()
		right_layout = QVBoxLayout()

		self.label_title = QLabel( "Motivational agent simulation" )
		self.label_title.setAlignment( Qt.AlignCenter )
		self.label_title.setFixedHeight( 20 )

		# Creating the painting context
		self.scanvas = SimulationCanvas( self, 500, 500 )		
		
		left_layout.addWidget( self.label_title )
		left_layout.addWidget( self.scanvas )
		left_layout.setContentsMargins( 0, 0, 0, 0 )
		left_layout.setSpacing( 0 )

		# Right panel
		pw = 350
		ph = 150

		# self.plotView1 = pg.widgets.RemoteGraphicsView.RemoteGraphicsView()
		# self.plotView1.setFixedWidth( pw+10 )
		# self.plotView1.pg.setConfigOptions(antialias=True)
	
		self.plotTop = pg.PlotWidget()
		self.plotTop.setFixedWidth( pw )
		self.plotTop.setFixedHeight( ph )
		self.plotTopData = self.plotTop.plot([0],[0])
		# self.plotView1.setCentralItem( self.plotTop )

		self.plotMiddle = pg.PlotWidget()
		self.plotMiddle.setFixedWidth( pw )
		self.plotMiddle.setFixedHeight( ph )
		self.plotMiddleData = self.plotMiddle.plot([0],[0])


		self.plotBottom = pg.PlotWidget()
		self.plotBottom.setFixedWidth( pw )
		self.plotBottom.setFixedHeight( ph )
		self.plotBottomData = self.plotBottom.plot([0],[0])

		right_layout.addWidget( self.plotTop )
		right_layout.addWidget( self.plotMiddle )
		right_layout.addWidget( self.plotBottom )
		right_layout.setContentsMargins( 0, 0, 0, 0 )

		# Building main layour
		main_layout.setContentsMargins( 0, 0, 0, 0 )
		main_layout.setSpacing( 0 )
		main_layout.addLayout( left_layout )		
		main_layout.addLayout( right_layout )
		
		self.widget = QWidget()
		self.widget.setLayout( main_layout )
		self.setCentralWidget( self.widget )

		toolbar = QToolBar( "Some toolbar" )
		self.addToolBar( toolbar )

		self.run_text = QLineEdit("10")
		self.run_text.setFixedWidth( 50 )
		self.run_action = QAction( "Run", self )
		self.run_action.setIcon( QIcon("icons/play.png") )
		self.run_action.setStatusTip( "Runs the simulation" )
		self.run_action.triggered.connect( self.onRunClick )

		self.pause_action = QAction( "Pause", self )
		self.pause_action.setIcon( QIcon("icons/pause.png") )
		self.pause_action.setStatusTip( "Pauses the simulation" )
		self.pause_action.triggered.connect( self.onPauseClick )
		self.pause_action.setEnabled( False )

		self.stop_action = QAction( "Stop", self )
		self.stop_action.setIcon( QIcon("icons/stop.png") )
		self.stop_action.setStatusTip( "Stops the simulation" )
		self.stop_action.triggered.connect( self.onStopClick )
		self.stop_action.setEnabled( False )

		self.plots_action = QAction( "Plots", self )
		self.plots_action.setIcon( QIcon("icons/plot.png") )
		self.plots_action.setStatusTip( "Shows additional plots" )
		self.plots_action.triggered.connect( self.onPlotsClick )

		self.screenshot_action = QAction( "Screenshot", self )
		self.screenshot_action.setStatusTip( "Saves screenshot" )
		self.screenshot_action.triggered.connect( self.onScreenShotClick )

		# self.plots_action.setEnabled( False )

		self.foodx = QLineEdit("-20")
		self.foodx.setFixedWidth( 50 )
		self.foody = QLineEdit("20")
		self.foody.setFixedWidth( 50 )

		toolbar.addAction( self.run_action )
		toolbar.addWidget( self.run_text )
		toolbar.addWidget( QLabel("ms") )
		toolbar.addAction( self.pause_action )
		toolbar.addAction( self.stop_action )
		toolbar.addAction( self.plots_action )
		toolbar.addAction( self.screenshot_action )
		toolbar.addWidget( QLabel("Food position:") )
		toolbar.addWidget( self.foodx )
		toolbar.addWidget( self.foody )

		self.statusb = QStatusBar( self )
		self.setStatusBar( self.statusb )

		self.simulation = None
		self.signals = GraphicSignals()		
		self.plots = PlotsWindow( self )

		self.agentPos = [20.0, 40.0]
		
	def onScreenShotClick( self, ev ):
		pixmap = QPixmap( self.widget.rect().width(), self.widget.rect().height() )
		painter = QPainter()
		painter.begin( pixmap )
		self.widget.render( painter, QPoint(), QRegion(self.widget.rect()) )
		painter.end()
		pixmap.save("screen" + str(datetime.datetime.now().strftime("%I_%M_%S")) + ".png")

	def onPlotsClick( self, ev ):
		self.plots.show()
		
	def showEvent( self, ev ):
		print( "main window shown")
		super(MainWindow, self).showEvent(ev)
		self.scanvas.initCanvas()


	def initPlots( self, model ):
		self.plotUpdater = PlotUpdater( self )
		# self.threadpool.start( self.plotUpdater )

		ao = model.getObservedAgent()

		self.topPlotData = {'name' : 'Rho'}
		self.middlePlotData = {'name' : 'Tb'}
		self.bottomPlotData = {'name' : 'E'}

		self.plotTop.setTitle( self.topPlotData['name'] )
		self.plotTop.setXRange( 0, self.simulation.maxT )
		self.plotTop.setYRange( -0.5, 1.5 )
		# Middle plot
		self.plotMiddle.setTitle( self.middlePlotData['name'] )
		vp = ao.brain.getPreferredValue(self.middlePlotData['name'])
		self.plotMiddle.plot([0, self.simulation.maxT],[vp, vp], pen = (5, 9))
		self.plotMiddle.setXRange( 0, self.simulation.maxT )
		self.plotMiddle.setYRange( 15, 50 )
		# Bottom plot
		self.plotBottom.setTitle( self.bottomPlotData['name'] )
		vp = ao.brain.getPreferredValue(self.bottomPlotData['name'])
		self.plotBottom.plot([0, self.simulation.maxT],[vp, vp], pen = (5, 9))
		self.plotBottom.setXRange( 0, self.simulation.maxT )
		self.plotBottom.setYRange( 0, 1.5 )

		# Initializing plots window
		if self.plots is not None:
			self.plots.populate(  model )


	def draw( self, model ):
		self.plotUpdater.collectData( model )
		self.scanvas.updateGraphics( model )
		pm = "(paused)" if self.simulation.is_pause else ""
		self.statusb.showMessage( "Simulation running t = " + str(self.simulation.t) + pm )

		self.plotUpdater.updatePlots( model )


	def start( self ):
		print( "Simulation started" )
		self.run_action.setEnabled( False )
		self.pause_action.setEnabled( True )
		self.stop_action.setEnabled( True )

	def finish( self ):
		self.statusb.showMessage( "Simulation finished" )
		self.run_action.setEnabled( True )
		self.pause_action.setEnabled( False )
		self.stop_action.setEnabled( False )
		self.plotUpdater.stop()

	def pause( self ):
		self.statusb.showMessage( "Simulation paused" )

	def onStopClick( self, s ):
		self.simulation.stop()

	def onPauseClick( self, s ):
		self.simulation.pause()

	def onRunClick( self, s ):	
		maxT = int(self.run_text.text())
		self.runSimulation( maxT )

	def runSimulation( self, maxT ):
		xmin = -self.scanvas.viewPort.getWidth()/2.0
		xmax = self.scanvas.viewPort.getWidth()/2.0
		
		self.simulation = Simulation( xmin, xmax, maxT )
		#a = AgentBuilder.buildSimpleTemperatureAgent( self.simulation.model.environment, 20.0, 20.0 )
		a = AgentBuilder.buildMotivationalAgent( self.simulation.model.environment, self.agentPos[0], self.agentPos[1] )		

		self.simulation.model.addAgent( a )
		self.simulation.model.addFoodSource( float(self.foodx.text()), float(self.foody.text()) )

		self.initPlots( self.simulation.model )

		self.simulation.signals.draw.connect( self.draw )
		self.simulation.signals.start.connect( self.start )
		self.simulation.signals.finish.connect( self.finish )
		self.simulation.signals.pause.connect( self.pause )
		self.signals.plottingCompleted.connect( self.simulation.resume )

		self.threadpool.start( self.simulation )

class PlotsWindow( QMainWindow ):
	def __init__( self, parent ):
		super( PlotsWindow, self ).__init__( parent )
		self.setFixedSize( 600, 400 )
		self.plot = pg.PlotWidget()
		self.plotDataItem = self.plot.plot([0],[0])
		self.combo = QComboBox()
		self.combo.currentIndexChanged.connect( self.comboChanged )
		layout = QVBoxLayout()
		layout.addWidget( self.plot )
		layout.addWidget( self.combo )
		lbl = QLabel()
		lbl.setLayout( layout )
		self.setCentralWidget(lbl)
		self.currentPlot = None

	def populate( self, model ):
		ao = model.getObservedAgent()
		variables = ao.brain.getAvailableVariables()
		self.combo.clear()

		if len(variables) > 0:
			self.combo.addItems( variables )
			self.currentPlot = variables[0]
			self.plot.setTitle( variables[0] )
		else:
			self.combo.addItem( "No variables to plot" )

	def comboChanged( self, ev ):
		self.currentPlot = self.combo.currentText()


class GraphicSignals( QObject ):
	plottingCompleted = pyqtSignal()

class PlotUpdater():
	def __init__( self, window ):
		# super( PlotUpdater, self ).__init__()
		self.model = None
		self.window = window
		self.pen = pg.mkPen( 'k', width=3.0 )
		self.plotting = False

	def updatePlots( self, model ):
		self.model = model

		if not self.plotting:
			self.plotting = True
			self.plotVariables( model )
			
	def collectData( self, model ):
		ao = model.getObservedAgent()
		recorder = ao.recorder.clone()
		self.x = recorder.getTime()
		self.y_top = recorder.getStateData( self.window.topPlotData['name'] )		
		self.y_middle = recorder.getStateData( self.window.middlePlotData['name'] )		
		self.y_bottom = recorder.getStateData( self.window.bottomPlotData['name'] )
		self.cstep = recorder.step

	def plotVariables( self, model ):			
		# y_var = ao.recorder.getVariableData( self.window.plots.currentPlot )
		#self.plotTop.clear()
		if self.cstep > 0:
			i1 = self.cstep-2
			i2 = self.cstep+1
			self.window.plotTop.plot(self.x[i1:i2], self.y_top[i1:i2], pen = self.pen, _callSync='off')
			QApplication.processEvents()
			#self.plotMiddle.clear()
			self.window.plotMiddle.plot(self.x[i1:i2], self.y_middle[i1:i2], pen = self.pen, _callSync='off')	
			QApplication.processEvents()
			self.window.plotBottom.plot(self.x[i1:i2], self.y_bottom[i1:i2], pen = self.pen, _callSync='off')
			QApplication.processEvents()

			# Plotting plots		
			# self.window.plots.plotDataItem.setData( x, y_var )
		self.plotting = False

	# def run( self ):
	# 	self.running = True

	# 	while self.running:
	# 		if self.model is not None:
	# 			self.plotVariables( self.model )
	# 			self.model = None

	# 	print( "Plot updater stopped" )

	def stop( self ):
		self.running = False


class SimulationCanvas( QWidget ): # To-do: Investigate QOpenGLWidget
	def __init__( self, parent, width, height ):
		super( SimulationCanvas, self ).__init__( parent )

		self.setAutoFillBackground( False )
		self.setFixedSize( width, height )
		self.viewPort = ViewPort( width, height )
		self.model = None
		self.agentPen = QPen()
		self.tracePen = QPen( QColor(100, 130, 100))
		self.agentColor = QColor(150, 128, 128)
		self.sensorsColor = QColor(50, 60, 80)
		self.linesPen = QPen()
		self.linesPen.setColor( QColor(230, 230, 230) )
		self.foodPen = QPen()
		self.foodPen.setWidth( 2 )
		# Initializing gradient
		self.gradient = QLinearGradient( 0, 0, self.viewPort.getWidth(), 0)
		self.gradient.setColorAt( 0.0, QColor(0,0,255) )
		self.gradient.setColorAt( 0.5, QColor(255,255,0) )
		self.gradient.setColorAt( 1.0, QColor(255,0,0) )

		self.marksPen = QPen()
		self.marksPen.setColor( QColor(0, 0, 0) )
		self.marksPen.setWidth( 2.0 )
		# fmt = QSurfaceFormat()
		# fmt.setSamples( 16 )
		# self.setFormat( fmt )

	def initCanvas( self ):
		self.pixmap = QPixmap("icons/apple.png")

	def updateGraphics( self, model ):
		self.model = model
		self.update()

	def drawEnvironment( self, painter, environment ):

		N = 10
		w = self.viewPort.getWidth()
		h = self.viewPort.getHeight()		

		for i in range( 1, N ):
			x1 = i*w/float(N)
			x2 = x1
			y1 = 0
			y2 = h
			
			painter.setPen( self.linesPen )
			painter.drawLine( x1, y1, x2, y2 )

			y1 = i*h/float(N)
			y2 = y1
			x1 = 0
			x2 = w
			painter.drawLine( x1, y1, x2, y2 )			

		
		# Drawing the food sources
		for (x,y) in environment.food_sources:
			xn, yn = self.viewPort.transformCoordinates( x, y )

			for i in range( 20 ):
				r = i*10
				xi = xn + r
				yi = yn
				c = (1-environment.g( xi, yi, xn, yn ))*255
				
				self.foodPen.setColor( QColor(c, c, c) ) 
				painter.setPen( self.foodPen )
				painter.drawEllipse( QRectF( xn-r/2.0, yn-r/2.0, r, r ) )

			painter.drawPixmap( xn-10, yn-10, 20, 20, self.pixmap )


	def drawAgents( self, painter, agents ):
		# Draw agents
		r = 20
		painter.setPen( self.agentPen )

		for a in agents:
			# Trace
			trace = a.recorder.getTrace()
			painter.setPen( self.tracePen )
			path = QPainterPath()

			m,n = trace.shape
			
			for i in range(n-1):
				x0 = trace[0,i]
				y0 = trace[1,i]
				x0,y0 = self.viewPort.transformCoordinates(x0, y0)
				x1 = trace[0,i+1]
				y1 = trace[1,i+1]
				x1,y1 = self.viewPort.transformCoordinates(x1, y1)
				path.moveTo( x0, y0 )
				path.lineTo( x1, y1 )

			painter.drawPath(path)
			# draw body
			x, y = a.body.getPosition()	
			x, y = self.viewPort.transformCoordinates( x, y )
			xc = x - r/2.0
			yc = y - r/2.0
			#graphics.trace( self.trace[0,:step], self.trace[1,:step] )
			self.agentPen.setWidth( 2.0 )
			painter.setPen( self.agentPen )
			painter.setBrush( self.agentColor )
			painter.drawEllipse( xc, yc, r, r )		

			rs = r/3.0
			self.agentPen.setWidth( 1.0 )
			painter.setBrush( self.sensorsColor )
			# Draw sensors
			for s in a.sensors:
				xs, ys = self.viewPort.transformCoordinates( s.pos[0], s.pos[1] )
				vx = xs - x
				vy = ys - y
				n = 2*np.sqrt( vx**2 + vy**2 )

				xns = x + r*vx/n
				yns = y + r*vy/n

				painter.drawEllipse( QRectF( xns - rs/2.0, yns - rs/2.0, rs, rs )) 


	def updateViewPort( self, agent ):
		x, y = agent.body.getPosition()	
		self.viewPort.update( x, y )
	
	def drawGradient( self, painter, environment ):
		
		painter.fillRect( 0, 0, self.viewPort.getWidth(), 10, self.gradient)

		N = 10
		w = self.viewPort.getWidth()
		h = self.viewPort.getHeight()
		
		for i in range( 1, N ):
			
			x1 = i*w/float(N)
			x2 = x1
			y1 = 0
			y2 = 20
			
			painter.setPen( self.marksPen )
			painter.drawLine( x1, y1, x2, y2 )

			font = painter.font()
			font.setPixelSize(10)
			painter.setFont( font )
			x,y = self.viewPort.translate( -w/2.0 + x1, 0 )

			T = environment.getTemperature( x, y )
			painter.drawText( x1 + 5, 20, "%.1f"%T )

	def paintEvent( self, event ):		

		painter = QPainter()
		painter.begin( self )
		painter.setRenderHint( QPainter.Antialiasing, True)
		painter.setRenderHint( QPainter.HighQualityAntialiasing, False)
		
		painter.eraseRect( 0, 0, self.viewPort.getWidth(), self.viewPort.getHeight() )
		painter.fillRect( event.rect(), Qt.white)
		
		pen = QPen()
		pen.setColor( QColor( 128, 128, 128 ) )
		painter.setPen( pen )
		painter.drawRect( 0, 0, self.viewPort.getWidth(), self.viewPort.getHeight())

		if self.model is not None:			
			

			ao = self.model.getObservedAgent()				

			if ao is not None:
				self.updateViewPort( ao )

			self.drawEnvironment( painter, self.model.environment )
			self.drawAgents( painter, self.model.agents )
			self.drawGradient( painter, self.model.environment )
			
		painter.end()


class ViewPort:
	def __init__(self, w, h ):
		self.vl = 0
		self.vr = w
		self.vb = h
		self.vt = 0

		self.margin = 100

	def getScreenCoordinates( self, x, y ):
		return x + self.getWidth()/2.0, -y + self.getHeight()/2.0

	def transformCoordinates( self, x, y ):	
		x,y = self.getScreenCoordinates(x, y)	
		return x - self.vl, y - self.vt

	def translate( self, x, y ):
		return x + self.vl, y + self.vt

	def update( self, x0, y0 ):
		""" Moves the viewport according to the positions of the agent """

		deltax = 0
		deltay = 0

		x0,y0 = self.getScreenCoordinates( x0, y0 )

		# x coordinate
		if self.vr < x0: # outside to the right
			deltax = x0 - self.vr + self.margin
		elif self.vl > x0:
			deltax = x0 - self.vl - self.margin

		if self.vt > y0: # outside above
			deltay = self.vt - y0 + self.margin
		elif self.vb < y0: # Outside at the bottom
			deltay = y0 - self.vb - self.margin

		self.vl += deltax
		self.vr += deltax
		self.vb -= deltay
		self.vt -= deltay

	def getWidth( self ):
		return self.vr - self.vl

	def getHeight( self ):
		return self.vb - self.vt


		

