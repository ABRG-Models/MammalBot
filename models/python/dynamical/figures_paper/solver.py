import numpy as np 

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