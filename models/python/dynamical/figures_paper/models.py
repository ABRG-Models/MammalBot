import numpy as np
from solver import Solver

# Model of two motivations in the original drive space
class TwoMotivations:
    def __init__(self) -> None:
        self.a1 = 2.0
        self.a2 = 2.0
        self.b1 = 30.0
        self.b2 = 30.0
        self.sigma = 10.0
        self.epsilon = 0.01    
        self.rho1 = -1.0
        self.rho2 = 1.0
        self.s = Solver()
        

    def integrate( self, T, q0, input_u = lambda t: 0.0, input_v = lambda t: 0.0 , output = None):        
        
        def F ( t, x, input_u = input_u, input_v = input_v, output = output ): 
            
            g1 = lambda q1, q2, rho: -self.a1*q1 + self.b1*(1.0 - q1)*np.exp(-self.sigma*(rho - self.rho1)**2)*(1.0 + input_u(t)) 
            g2 = lambda q1, q2, rho: -self.a2*q2 + self.b2*(1.0 - q2)*np.exp(-self.sigma*(rho - self.rho2)**2)*(1.0 + input_v(t)) 
            f = lambda q1, q2, rho: -4.0*((rho - self.rho1)*(rho - self.rho2)*(rho - (self.rho1+self.rho2)/2.0) + 
                                        (1-q1)*(rho - self.rho1)/2.0 + (1-q2)*(rho - self.rho2)/2.0)

            if output is not None:
                output(t, x)

            return np.array([self.epsilon*g1(x[0], x[1], x[2]),
                      self.epsilon*g2(x[0], x[1], x[2]),
                      f(x[0], x[1], x[2])])


        t, X = self.s.integrate( F, T, q0 )
        return t, X

# Model of two motivations in the arousal-valence space
class TwoMotivationsRotated:
    def __init__(self) -> None:
        self.a = 2.0
        self.b = 30.0
        self.sigma = 10.0
        self.epsilon = 0.01    
        self.s = Solver()
        

    def integrate( self, T, x0 ):
        g1 = lambda x, y, z: -self.a*x + self.b*np.exp(-self.sigma*(z**2 + 1))*((2.0 - x)*np.cosh(2.0*self.sigma*z) + y*np.sinh(2.0*self.sigma*z)) 
        g2 = lambda x, y, z: -self.a*y - self.b*np.exp(-self.sigma*(z**2 + 1))*(y*np.cosh(2.0*self.sigma*z) + (2.0 - x)*np.sinh(2.0*self.sigma*z))
        f = lambda x, y, z: -4.0*(z**3 - x*z/2.0 - y/2.0)

        F = lambda t, x: np.array([self.epsilon*g1(x[0], x[1], x[2]),
                                   self.epsilon*g2(x[0], x[1], x[2]),
                                   f(x[0], x[1], x[2])])


        t, X = self.s.integrate( F, T, x0 )
        return t, X

# Model of two motivations in the Hesse normal form space
class TwoMotivationsHesse:
    def __init__(self) -> None:
        self.a = 2.0
        self.b = 30.0
        self.sigma = 10.0
        self.epsilon = 0.01    
        self.s = Solver()
        

    def integrate( self, T, x0 ):
        
        g1 = lambda P, phi, r: -self.a*P - self.b*np.exp(-self.sigma*(r**2 + 1))*(P*np.cosh(2.0*self.sigma*r) + (2.0*np.sin(phi) + np.cos(phi))*np.sinh(2.0*self.sigma*r)) + (P/np.tan(phi))*(self.a*np.cos(phi) - self.b*np.exp(-self.sigma*(r**2+1.0))*((2.0*np.sin(phi)**2 + np.sin(phi)*np.cos(phi))*np.cosh(2.0*self.sigma*r)) + P*np.sin(phi)*np.sinh(2.0*self.sigma*r))
        g2 = lambda P, phi, r: (np.cos(phi)/(np.tan(phi)*(np.cos(phi)**2 - np.sin(phi)**2)))*(self.a*np.cos(phi) + self.b*np.exp(-self.sigma*(r**2 + 1.0))*((2.0*np.sin(phi) + np.cos(phi))*np.cosh(2.0*self.sigma*r) + P*np.sinh(2.0*self.sigma*r)))
        f = lambda P, phi, r: -4.0*(r*np.cos(phi) + r**3*np.sin(phi) - P)

        F = lambda t, x: np.array([self.epsilon*g1(x[0], x[1], x[2]),
                                   self.epsilon*g2(x[0], x[1], x[2]),
                                   f(x[0], x[1], x[2])])

        
        t, X = self.s.integrate( F, T, x0 )
        return t, X

class TwoMotivationsDiscrete:
    def __init__(self) -> None:
        self.a1 = 2.0
        self.a2 = 2.0
        self.b1 = 30.0
        self.b2 = 30.0
        self.sigma = 10.0
        self.epsilon = 0.005    
        self.rho1 = -1.0
        self.rho2 = 1.0
        self.eta = lambda r: np.abs(r) < 1.0/self.sigma
        self.s = Solver()
        

    def integrate( self, T, x0 ):
        g1 = lambda q1, q2, rho: -self.a1*q1 + self.b1*(1.0 - q1)*self.eta(rho - self.rho1)
        g2 = lambda q1, q2, rho: -self.a2*q2 + self.b2*(1.0 - q2)*self.eta(rho - self.rho2) 
        f = lambda q1, q2, rho: -2.0*((rho - self.rho1)*(rho - self.rho2)*(rho - (self.rho1+self.rho2)/2.0) + 
                                      (1-q1)*(rho - self.rho1)/2.0 + (1-q2)*(rho - self.rho2)/2.0)

        F = lambda t, x: np.array([self.epsilon*g1(x[0], x[1], x[2]),
                                   self.epsilon*g2(x[0], x[1], x[2]),
                                   f(x[0], x[1], x[2])])


        
        
        t, X = self.s.integrate( F, T, x0 )
        return t, X