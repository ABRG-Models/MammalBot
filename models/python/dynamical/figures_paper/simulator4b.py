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
        g2 = lambda q1, q2, rho: -self.a2*q2 + self.b2*(1.0 - q2)*np.exp(-self.sigma*(rho - self.rho2)**2)*(1.0 + input_v) 
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
        self.interruptions = []

        # Motivational kernels
        self.eta1 = lambda x: np.heaviside( -x, 0.0)
        self.eta2 = lambda x: np.heaviside(x, 0.0)
        

    def run(self, x0, T = 1000):
        self.X = np.zeros((len(x0), T))
        self.X[:,0] = x0
        self.time = np.zeros(T)
        K = 0.1
        flag = False
        int_time = -1
        duration = 105

        for i in range(T-1):
            

            input1, input2 = self.world.step(self.time[i], 
                                             self.eta1(self.X[2,i]), 
                                             self.eta2(self.X[2,i]))
            
            self.X[:, i+1] = self.model.step(self.time[i], self.X[:,i], input1, input2)
            self.time[i+1] = self.time[i] + self.h
            
            
        return self.time, self.X
        


class World:
    def __init__( self, h):
        self.h = h
        self.t0 = 0
        

    def consume( self, t, eta1, eta2 ):
        NOP
        
    def pursue( self, t, eta1, eta2 ):
        NOP
    
    def search( self, t, eta1, eta2 ):
        NOP

    def step(self, t, eta1, eta2):


        input1, input2 = self.consume(t, eta1, eta2)

        

       
        return input1, input2

