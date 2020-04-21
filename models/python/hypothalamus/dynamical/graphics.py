from PyQt4 import QtGui
import pyqtgraph as pg

class Graphics:

	def __init__( self, w, h ):
		self.vxmin = 0
		self.vymin = 0
		self.vxmax = w
		self.vymax = h


	def initLayout( self ):
		# Init main window
		app = QtGui.QApplication([])

		# Init main arena

		# Init gradient

		# Init other axes

		# Init other windows


	def updateViewport( self, xc, yc, w, h ):
	# If the agent goes out of view, then move the viewport

		if x + self.offset[0] > self.environment.w :
			self.offset[0] = self.environment.w + self.offset[0] - x - 10.0
		if y + self.offset[1] > self.environment.h:
			self.offset[1] = self.environment.h + self.offset[1] - y - 10.0
		if x + self.offset[0] < 0:
			self.offset[0] = -x + self.offset[0] + 10.0
		if y + self.offset[1] < 0:
			self.offset[1] = -y + self.offset[1] + 10.0

	plt.sca(self.ax)
		plt.cla()
		plt.axis([0, self.environment.w, 0, self.environment.h])
		#plt.title('Motivational conflict motivation')
		plt.xlabel('x')
		plt.ylabel('y')
		# Setting figure properties
		plt.ion()		


	def drawGradient( self ):
		cax = plt.gca()

		if self.ax_gradient is None:
			self.ax_gradient = plt.axes([0.07, 0.83, 0.4, 0.09])	

		plt.sca( self.ax_gradient )
		self.ax_gradient.tick_params(bottom = False, left = True, top = False, labelbottom = False, labelleft = True)
		
		self.ax_gradient.axis([0, self.w, self.Tmin, self.Tmax])
		plt.yticks([self.Tmin, self.Tmax], [self.Tmin, self.Tmax])
		xx = [0 + offset[0], self.w + offset[0], self.w + offset[0]]
		yy = [self.Tmin + offset[0], self.Tmin + offset[0], self.Tmax + offset[0]]
		self.ax_gradient.fill( xx, yy, color=[0.8,0.5,0.5] )

	def drawFoodSource( self, x0, y0 ):
		plt.sca( self.ax_gradient )
			#self.ax_gradient.axis([0, self.w, self.Tmin, self.Tmax])
			plt.yticks([self.Tmin, self.Tmax], [self.Tmin - offset[0], self.Tmax - offset[0]])

			plt.sca(cax)
			
			x0 += offset[0]
			y0 += offset[1]

			x = np.arange(0 + offset[0], self.w + offset[0], delta)
			y = np.arange(0 + offset[1], self.h + offset[1], delta)

			X, Y = np.meshgrid(x, y)
			#Z = np.exp(-(X - x0*np.ones(X.shape))**2 - (Y - y0*np.ones(Y.shape))**2)
			Z = self.g(X,Y, x0*np.ones(X.shape), y0*np.ones(Y.shape))		
			CS = plt.contour(X, Y, Z, origin = 'lower' )

			c = plt.Circle( (x0, y0), 1.0, color = 'g' )
			fig = plt.gcf()
			ax = fig.gca()
			ax.add_artist(c)

	def plot( self, xdata, ydata, title ):
		if step > 0:
			ax.plot(time[step-1:step+1], self.state[idx,step-1:step+1], 'k', linewidth = 3.0)

		if axis is not None:
			ax.axis(axis)	
			
		ax.set_title(name)

	def circle( self, x, y, radius, color = 'k' ):
		c = plt.Circle( (x, y), self.body.radius, color = 'k' )	
		ax.add_artist(c)

	def trace( self ):
		( self.trace[0,:step], self.trace[1,:step], 'k--')

	def plotPotential( self, U, rho ):