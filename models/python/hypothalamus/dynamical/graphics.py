import pyqtgraph as pg

import sys, os
from PyQt5.QtCore import Qt, QThreadPool, QRectF, pyqtSlot
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QHBoxLayout, QVBoxLayout, QToolBar, QWidget, QAction, QStatusBar, QOpenGLWidget
from PyQt5.QtGui import QPen, QBrush, QColor, QPixmap, QPainter, QSurfaceFormat, QLinearGradient, QIcon, QLineEdit, QPainterPath

from simulation import *
import time 

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

		self.plotTop = pg.PlotWidget()
		self.plotTop.setFixedWidth( pw )
		self.plotTop.setFixedHeight( ph )
		self.plotTopData = self.plotTop.plot([0],[0])

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

		# Building main layour
		main_layout.setContentsMargins( 0, 0, 0, 0 )
		main_layout.setSpacing( 0 )
		main_layout.addLayout( left_layout )		
		main_layout.addLayout( right_layout )
		
		widget = QWidget()
		widget.setLayout( main_layout )
		self.setCentralWidget( widget )

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

		toolbar.addAction( self.run_action )
		toolbar.addWidget( self.run_text )
		toolbar.addWidget( QLabel("ms") )
		toolbar.addAction( self.pause_action )
		toolbar.addAction( self.stop_action )

		self.statusb = QStatusBar( self )
		self.setStatusBar( self.statusb )

		self.simulation = None
		self.signals = GraphicSignals()
		self.plotUpdater = PlotUpdater( self )
		self.threadpool.start( self. plotUpdater )
		
	def showEvent( self, ev ):
		super(MainWindow, self).showEvent(ev)
		self.runSimulation(0.2)


	def initPlots( self ):
		self.topPlotData = {'name' : 'Rho'}
		self.middlePlotData = {'name' : 'Tb'}
		self.bottomPlotData = {'name' : 'E'}
		self.plotTop.setTitle( self.topPlotData['name'] )
		self.plotTop.setXRange( 0, self.simulation.maxT )
		self.plotTop.setYRange( -0.5, 1.5 )
		self.plotMiddle.setTitle( self.middlePlotData['name'] )
		self.plotMiddle.setXRange( 0, self.simulation.maxT )
		self.plotMiddle.setYRange( 15, 50 )
		self.plotBottom.setTitle( self.bottomPlotData['name'] )
		self.plotBottom.setXRange( 0, self.simulation.maxT )
		self.plotBottom.setYRange( 0, 1.5 )

	def draw( self, model ):
		self.scanvas.updateGraphics( model )
		pm = "(paused)" if self.simulation.is_pause else ""
		self.statusb.showMessage( "Simulation running t = " + str(self.simulation.t) + pm )

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
		a = AgentBuilder.buildMotivationalAgent( self.simulation.model.environment, 20.0, 20.0 )		

		self.simulation.model.addAgent( a )
		self.simulation.model.addFoodSource(  -0.0, 0.0  )

		self.simulation.signals.draw.connect( self.draw )
		self.simulation.signals.draw.connect( self.plotUpdater.updatePlots )
		self.simulation.signals.start.connect( self.start )
		self.simulation.signals.finish.connect( self.finish )
		self.simulation.signals.pause.connect( self.pause )

		self.signals.plottingCompleted.connect( self.simulation.resume )

		self.initPlots()
		self.threadpool.start( self.simulation )

class GraphicSignals( QObject ):
	plottingCompleted = pyqtSignal()

class PlotUpdater( QRunnable ):
	def __init__( self, window ):
		super( PlotUpdater, self ).__init__()
		self.model = None
		self.window = window

	def updatePlots( self, model ):
		self.model = model

	def plotVariables( self, model ):
		ao = model.getObservedAgent()

		x = ao.recorder.getTime()
		y_top = ao.recorder.getStateData( self.window.topPlotData['name'] )		
		y_middle = ao.recorder.getStateData( self.window.middlePlotData['name'] )		
		y_bottom = ao.recorder.getStateData( self.window.bottomPlotData['name'] )	
		#self.plotTop.clear()
		self.window.plotTopData.setData(x, y_top)
		QApplication.processEvents()

		#self.plotMiddle.clear()
		self.window.plotMiddleData.setData(x, y_middle)
		QApplication.processEvents()

		#self.plotBottom.clear()
		self.window.plotBottomData.setData(x, y_bottom)
		QApplication.processEvents()

	def run( self ):
		while True:
			if self.model is not None:
				self.plotVariables( self.model )
				self.model = None


class SimulationCanvas( QOpenGLWidget ):
	def __init__( self, parent, width, height ):
		super( SimulationCanvas, self ).__init__( parent )

		self.setAutoFillBackground( False )
		self.setFixedSize( width, height )
		self.viewPort = ViewPort( width, height )
		self.model = None

		fmt = QSurfaceFormat()
		fmt.setSamples( 8 )
		self.setFormat( fmt )

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
			pen = QPen()
			pen.setColor( QColor(230, 230, 230) )
			painter.setPen( pen )
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
				pen = QPen()
				pen.setWidth( 2 )
				pen.setColor( QColor(c, c, c) ) 
				painter.setPen( pen )
				painter.drawEllipse( QRectF( xn-r/2.0, yn-r/2.0, r, r ) )

			painter.drawPixmap( xn-10, yn-10, 20, 20, self.pixmap )


	def drawAgents( self, painter, agents ):
		# Draw agents
		r = 20
		pen = QPen()
		painter.setPen( pen )

		for a in agents:
			# Trace
			trace = a.recorder.getTrace()
			painter.setPen( QPen( QColor(100, 130, 100), 1, Qt.DashLine) )
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
			pen.setWidth( 2.0 )
			painter.setPen( pen )
			painter.setBrush( QColor(150, 128, 128) )
			painter.drawEllipse( xc, yc, r, r )		

			rs = r/3.0
			pen.setWidth( 1.0 )
			painter.setBrush( QColor(50, 60, 80) )
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
		gradient = QLinearGradient( 0, 0, self.viewPort.getWidth(), 0)
		gradient.setColorAt( 0.0, QColor(0,0,255) )
		gradient.setColorAt( 0.5, QColor(255,255,0) )
		gradient.setColorAt( 1.0, QColor(255,0,0) )

		painter.fillRect( 0, 0, self.viewPort.getWidth(), 10, gradient)

		N = 10
		w = self.viewPort.getWidth()
		h = self.viewPort.getHeight()
		

		for i in range( 1, N ):
			
			x1 = i*w/float(N)
			x2 = x1
			y1 = 0
			y2 = 20
			pen = QPen()
			pen.setColor( QColor(0, 0, 0) )
			pen.setWidth( 2.0 )
			painter.setPen( pen )
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
		painter.fillRect( event.rect(), QColor(255,255,255))
		
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


		

