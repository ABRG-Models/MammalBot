import pyqtgraph as pg

import sys, os
from PyQt5.QtCore import Qt, QThreadPool, QRectF
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
		toolbar.addWidget(self.run_text)
		toolbar.addWidget(QLabel("ms"))
		toolbar.addAction( self.pause_action )
		toolbar.addAction( self.stop_action )

		self.setStatusBar( QStatusBar( self ) )

		self.simulation = None
		self.initPlots()

	def initPlots( self ):
		self.topPlotData = {'name' : 'Rho'}
		self.middlePlotData = {'name' : 'Tb'}
		self.bottomPlotData = {'name' : 'E'}
		self.plotTop.setTitle( self.topPlotData['name'] )
		self.plotMiddle.setTitle( self.middlePlotData['name'] )
		self.plotBottom.setTitle( self.bottomPlotData['name'] )

	def draw( self, model ):
		self.scanvas.updateGraphics( model )

	def plotVariables( self, model ):
		
		ao = model.getObservedAgent()

		x = ao.recorder.getTime()
		y = ao.recorder.getStateData( self.topPlotData['name'] )		
		#self.plotTop.clear()
		self.plotTopData.setData(x, y)

		x = ao.recorder.getTime()
		y = ao.recorder.getStateData( self.middlePlotData['name'] )		
		#self.plotMiddle.clear()
		self.plotMiddleData.setData(x, y)

		x = ao.recorder.getTime()
		y = ao.recorder.getStateData( self.bottomPlotData['name'] )	
		#self.plotBottom.clear()
		self.plotBottomData.setData(x, y)

	def start( self ):
		print( "Simulation started" )
		self.run_action.setEnabled( False )
		self.pause_action.setEnabled( True )
		self.stop_action.setEnabled( True )

	def finish( self ):
		print( "Simulation finished" )
		self.run_action.setEnabled( True )
		self.pause_action.setEnabled( False )
		self.stop_action.setEnabled( False )

	def pause( self ):
		print( "Simulation paused" )

	def onStopClick( self, s ):
		self.simulation.stop()

	def onPauseClick( self, s ):
		self.simulation.pause()

	def onRunClick( self, s ):	
		xmin = -self.scanvas.viewPort.getWidth()/2.0
		xmax = self.scanvas.viewPort.getWidth()/2.0
		maxT = int(self.run_text.text())
		self.simulation = Simulation( xmin, xmax, maxT )
		#a = AgentBuilder.buildSimpleTemperatureAgent( self.simulation.model.environment, 20.0, 20.0 )
		a = AgentBuilder.buildMotivationalAgent( self.simulation.model.environment, 20.0, 20.0 )		

		self.simulation.model.addAgent( a )
		self.simulation.model.addFoodSource(  0.0, 0.0  )

		self.simulation.signals.draw.connect( self.draw )
		self.simulation.signals.draw.connect( self.plotVariables )
		self.simulation.signals.start.connect( self.start )
		self.simulation.signals.finish.connect( self.finish )
		self.simulation.signals.pause.connect( self.pause )

		self.threadpool.start( self.simulation )


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
				r = i*7
				xi = xn + r
				yi = yn
				c = (1-environment.g( xi, yi, xn, yn ))*255
				pen = QPen()
				pen.setWidth( 2 )
				pen.setColor( QColor(c, c, c) ) 
				painter.setPen( pen )
				painter.drawEllipse( QRectF( xn-r/2.0, yn-r/2.0, r, r ) ) 


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


		

