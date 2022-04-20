import numpy as np
import matplotlib.pyplot as plt

# Integration scheme
class Solver:
    def __init__(self) -> None:
        self.h = 0.005

    def integrate( self, f, T, x0 ):
        X = np.zeros((len(x0), T))
        time = np.zeros(T)
        X[:,0] = x0

        for i in range(T-1):
            k1 = f(time[i], X[:,i])
            k2 = f(time[i] + self.h/2.0, X[:,i] + self.h*k1/2.0)
            k3 = f(time[i] + self.h/2.0, X[:,i] + self.h*k2/2.0)
            k4 = f(time[i] + self.h, X[:,i] + self.h*k3)
            X[:, i+1] = X[:,i] + self.h*(k1 + 2*k2 + 2*k3 + k4)/6.0
            time[i+1] = time[i] + self.h
            
        return time, X



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

T = 100000
model = TwoMotivations()
model.b2 =model.b1 = 20.0
x_agent = [-1.5]
x_food = -2.0
x_water = 2.0
b_sigma = 20.0

def input_u( t ):
    # return 0.0
    food = np.abs(x_agent[-1] - x_food)

    if food < 0.5:
        print("eating")
        return 2000.0
    else:
        return 0.0


def input_v( t ):
    water = np.abs(x_agent[-1] - x_water)

    if water < 0.5:
        print("drinking")
        return 2000.0
    else:
        return 0.0

def output(t, x):
    K1 = np.exp(-b_sigma*(x[2] - model.rho1)**2)
    K2 = np.exp(-b_sigma*(x[2] - model.rho2)**2)
    dG1 = -2.0*(x_agent[-1] - x_food)*np.exp(-1.0*(x_agent[-1] - x_food)**2)
    dG2 = -2.0*(x_agent[-1] - x_water)*np.exp(-1.0*(x_agent[-1] - x_water)**2)

    x_agent_new = x_agent[-1]
    if( np.abs(x[2] - model.rho1) < 0.5 ):
        x_agent_new = x_agent[-1] - 1.0*(K1*dG1 + (np.random.rand()-0.5)*0.00)
    elif( np.abs(x[2] - model.rho2) < 0.5 ):
        x_agent_new = x_agent[-1] - 1.0*(K2*dG2 + (np.random.rand()-0.5)*0.00)
    
    x_agent.append(x_agent_new)


t, X = model.integrate(T, [0.0, 1.0, 1.0], input_u, input_v, output)

# for i in range(100000):

#     output(0.0, [0, 0, -1.0])

plt.figure()
x = np.linspace(-2.5, 2.5, 100)
G1 = np.exp(-1.0*(x - x_food)**2)
G2 = np.exp(-1.0*(x - x_water)**2)
plt.plot(x,G1)
plt.plot(x, G2)
plt.figure()
plt.plot( x_agent )

plt.figure()
plt.plot(t, X[2, :])
plt.show()


    