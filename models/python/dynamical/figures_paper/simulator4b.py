from telnetlib import NOP
import numpy as np

class Model:

    def __init__(self, h) -> None:
        self.a1 = 2.0
        self.a2 = 2.0
        self.b1 = 30.0
        self.b2 = 30.0
        self.sigma = 10.0
        self.epsilon = 0.01    
        self.rho1 = -1.0
        self.rho2 = 1.0
        self.h = h
        

    def map( self, t, x, input_u = 0, input_v = 0): 
            
        g1 = lambda q1, q2, rho: -self.a1*q1 + self.b1*(1.0 - q1)*np.exp(-self.sigma*(rho - self.rho1)**2)*(1.0 + input_u) 
        g2 = lambda q1, q2, rho: -self.a2*(q2 - input_v) + self.b2*(1.0 - q2)*np.exp(-self.sigma*(rho - self.rho2)**2)*(1.0 + np.abs(input_v)) 
        f = lambda q1, q2, rho: -4.0*((rho - self.rho1)*(rho - self.rho2)*(rho - (self.rho1+self.rho2)/2.0) + 
                                    (1-q1)*(rho - self.rho1)/2.0 + (1-q2)*(rho - self.rho2)/2.0)

        return np.array([self.epsilon*g1(x[0], x[1], x[2]),
                    self.epsilon*g2(x[0], x[1], x[2]),
                    f(x[0], x[1], x[2])])



    def step(self, t, X, input1, input2):
        f = lambda t, x: self.map(t, x, input1, input2)
        k1 = f(t, X)
        k2 = f(t + self.h/2.0, X + self.h*k1/2.0)
        k3 = f(t + self.h/2.0, X + self.h*k2/2.0)
        k4 = f(t + self.h, X + self.h*k3)
        return  X + self.h*(k1 + 2*k2 + 2*k3 + k4)/6.0


class Experiment:
    def __init__(self, model, world, h) -> None:
        self.model = model
        self.world = world
        self.h = h

        # Motivational kernels
        self.eta1 = lambda x: np.heaviside( -x, 0.0 )
        self.eta2 = lambda x: np.heaviside( x, 0.0 )
        

    def run(self, x0, T = 1000):
        self.X = np.zeros((len(x0), T))
        self.X[:,0] = x0
        self.time = np.zeros(T)
        
        self.pos = np.zeros(T)

        for i in range(T-1):
            input1, input2 = self.world.step(self.time[i], 
                                             self.eta1(self.X[2,i]), 
                                             self.eta2(self.X[2,i]))
            # input1, input2 = self.world.step(self.time[i], 
            #                                  1.0, 
            #                                  0.0)
            print("input1: %.1f, input2: %.1f"%(input1, input2))
            self.X[:, i+1] = self.model.step(self.time[i], self.X[:,i], input1, input2)
            self.time[i+1] = self.time[i] + self.h
            self.pos[i+1] = self.world.x
            
        return self.time, self.X, self.pos
        
class HungerMotivation:
    def __init__(self, world):
        self.world = world
        self.state = self.search

    def consume( self, t, eta ):
        if eta > 0.5:
            return 10
        else:
            self.state = self.search
            return 0
        
    def pursue( self, t, eta ):
        g = self.world.gradientFood()
        self.world.move( -eta*g )

        if self.world.reachedFood():
            self.state = self.consume
        
        return 0
    
    def search( self, t, eta ):
        g = self.world.gradientFood()

        if g < 0.01:
            self.world.move( -eta*np.random.rand() )
        else:
            self.state = self.pursue
        
        return 0

    def step( self, t, eta ):
        return self.state(t, eta)


class TemperatureMotivation:
    def __init__(self, world):
        self.world = world
        self.state = self.search

    def consume( self, t, eta ):
        if eta < 0.5:
            self.state = self.search
        return self.world.temperature()
        
    def pursue( self, t, eta ):
        g = self.world.gradientTemperature()
        self.world.move( eta*g )

        if self.world.reachedSweetSpot():
            self.state = self.consume
        
        return self.world.temperature()
    
    def search( self, t, eta ):
        g = self.world.gradientTemperature()

        if g < 0.01:
            self.world.move( -eta*np.random.randn() )
        else:
            self.state = self.pursue
        
        return self.world.temperature()

    def step( self, t, eta ):
        return self.state(t, eta)

class World:
    def __init__( self, h):
        self.h = h
        self.tmin = -2.0
        self.hungerMotivation = HungerMotivation(self)
        self.temperatureMotivation = TemperatureMotivation(self)
        self.foodSignal = lambda x: np.exp(-(x + 1.0)**2)
        self.tSignal = lambda x: -self.tmin*x/2.0 + self.tmin/2.0
        self.x = 0.0

    def gradientFood( self ):
        return -2.0*(self.x + 1.0)*np.exp(-(self.x + 1.0)**2.0)

    def gradientTemperature( self ):
        return 5.0
    
    def temperature( self ):
        return self.tSignal(self.x)

    def reachedFood( self ):
        return np.abs(self.x + 1.0) < 0.1

    def reachedSweetSpot( self ):
        return np.abs(self.x - 1.0) < 0.1

    def move( self, g ):
        if np.abs(self.x) < 1.2:
            self.x += 0.01*g

    def step(self, t, eta1, eta2):

        input1 = self.hungerMotivation.step(t, eta1)
        input2 = self.temperatureMotivation.step(t, eta2)
      
        return input1, input2

