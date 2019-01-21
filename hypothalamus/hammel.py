import numpy as np
import pylab as pl

class sim:

    def __init__(self,timeStop, dt):
        self.timeStop = timeStop
        self.dt = dt
        self.time = 0.
        self.steps = 0
        self.connections = []
        self.neurons = []

    '''
    def reset(self):
        self.time = 0.
        self.steps = 0
    '''

    def step(self):
        self.steps +=1
        self.time += self.dt


class PSC:

    def __init__(self, sim, tau1, tau2, gmax, rev, ton):
        self.sim = sim
        self.tau1 = tau1
        self.tau2 = tau2
        self.gmax = gmax
        self.ton = ton
        self.tauRise = tau1*tau2/(tau1-tau2)
        self.G = gmax/(pow(tau2/tau1,self.tauRise/tau1)-pow(tau2/tau1,self.tauRise/tau2))
        self.rev = rev

    def step(self):
        trel = self.sim.time - self.ton
        if(trel>0.):
            return self.G*(np.exp(-trel/self.tau1)-np.exp(-trel/self.tau2))
        else:
            return 0.


class neuron:

    def __init__(self, sim, id, Vreset,Vthres,EL,gL,tauM  ,tau1,tau2,gmax,rev):
        self.sim = sim
        self.id = id
        self.Vreset = Vreset            # resting (and reset) potential in mV
        self.Vthres = Vthres            # spiking threshold in mV
        self.EL = EL
        self.gL = gL                    # Leak conductance (in mS/cm^2)
        self.Rm = 1./self.gL            # membrane resistance (in Mega Ohms)
        self.tauM = tauM                # membrane time constant (in seconds)
        self.overTauM = 1./self.tauM
        self.V = Vreset                 # membrane potential (mV)
        self.PSCs = []
        self.Tspike = []
        self.P = []
        self.sim.connections.append([])
        self.Vrecord = []

        self.tau1=tau1
        self.tau2=tau2
        self.gmax=gmax
        self.rev=rev

    # post-synaptic current
    def step(self):

        # compute current input
        Iin = 0.
        for p in self.PSCs:
            Iin += p.step()*(self.V-p.rev)
        self.P = np.hstack([self.P,Iin])

        # spike
        if(self.V>self.Vthres):
            self.V = self.Vreset
            self.Tspike = np.hstack([self.Tspike,self.sim.time])
            self.Vrecord = np.hstack([self.Vrecord,0.])
            for output in self.sim.connections[self.id]:
                self.sim.neurons[output].PSCs.append(PSC(self.sim,self.tau1,self.tau2,self.gmax,self.rev,self.sim.time))
        else:
            self.V += self.sim.dt*( self.EL - self.V - self.Rm * Iin ) * self.overTauM
            self.Vrecord = np.hstack([self.Vrecord,self.V])




# PARAMS

# Excitatory PSC params (AMPA)
Ex_tau1 = 0.00100       # time constant 1 (seconds)
Ex_tau2 = 0.00022       # time constant 2 (seconds)
Ex_rev = 0.             # reversal potential (mV)
Ex_gmax = 0.014         # maximum amplitude (in mS/cm^2)

# Inhibitory PSC params (GABA)
In_tau1 = 0.0040        # time constant 1 (seconds)
In_tau2 = 0.0030        # time constant 2 (seconds)
In_rev = -85.           # reversal potential (mV)
In_gmax = 0.028         # maximum amplitude (in mS/cm^2)

# Neuron params (cortex)
Vreset = -70.
Vthres = -65.
EL = -69.
gL = 0.03
tauM = 0.012


# NETWORK

# JACK TO EDIT/EXPLORE THESE VALUES *****
rate1 = 200                 # Tonic inhibition of WARM neuron 2
rate2 = 200                 # Tonic excitation of COOL neuron 3
num_temperatures = 12        # number to test (on x-axis)
duration = 2                # number of seconds
# JACK TO EDIT/EXPLORE THESE VALUES *****


max_temperature = 40.   # degrees C
max_firingRate = 1000.  # theoretical maximum firing rate of our neuron

temperatures = np.linspace(0,max_temperature,num_temperatures)

input_rates = max_firingRate * temperatures / max_temperature

NEURON2rate = np.zeros(len(temperatures))
NEURON3rate = np.zeros(len(temperatures))

for i,rate0 in enumerate(input_rates):

    # initialize with duration and dt
    S = sim(duration, 0.0001)

    # Warm-sensitive
    S.neurons.append(neuron(S,0,Vreset,Vthres,EL,gL,tauM,  Ex_tau1,Ex_tau2,Ex_gmax,Ex_rev)) # Excitatory
    S.neurons.append(neuron(S,1,Vreset,Vthres,EL,gL,tauM,  In_tau1,In_tau2,In_gmax,In_rev)) # Inhibitory

    # 'WARM' neuron - triggering cooling
    S.neurons.append(neuron(S,2,Vreset,Vthres,EL,gL,tauM,  Ex_tau1,Ex_tau2,Ex_gmax,Ex_rev)) # Excitatory

    # 'COOL' neuron - triggering warming
    S.neurons.append(neuron(S,3,Vreset,Vthres,EL,gL,tauM,  Ex_tau1,Ex_tau2,Ex_gmax,Ex_rev)) # Excitatory

    # network connections (from indexed neuron to appended neuron)
    S.connections[0].append(2)
    S.connections[1].append(3)

    #rate0 = 200     # Should increase with temperature (calibrated to 100)

    # SIMULATION
    while(S.time<S.timeStop):

        # Single Poisson input from spinal cord to warm-sensitive neurons
        if(rate0*S.dt>np.random.rand()):
            S.neurons[0].PSCs.append(PSC(S, Ex_tau1,Ex_tau2,Ex_gmax,Ex_rev,S.time))
            S.neurons[1].PSCs.append(PSC(S, Ex_tau1,Ex_tau2,Ex_gmax,Ex_rev,S.time))

        # Tonic inhibition of WARM neuron 2
        if(rate1*S.dt>np.random.rand()):
            S.neurons[2].PSCs.append(PSC(S, In_tau1,In_tau2,In_gmax,In_rev,S.time))

        # Tonic excitation of COOL neuron 3
        if(rate2*S.dt>np.random.rand()):
            S.neurons[3].PSCs.append(PSC(S, Ex_tau1,Ex_tau2,Ex_gmax,Ex_rev,S.time))

        for n in S.neurons:
            n.step()
        S.step()


    NEURON2rate[i] = 1.*len(S.neurons[2].Tspike)/S.timeStop
    NEURON3rate[i] = 1.*len(S.neurons[3].Tspike)/S.timeStop

    print ('input rate: ' + str(rate0))
    print ('spikes for neuron 0: '+str(len(S.neurons[0].Tspike)) + ', spike rate = '+str(1.*len(S.neurons[0].Tspike)/S.timeStop))
    print ('spikes for neuron 1: '+str(len(S.neurons[1].Tspike)) + ', spike rate = '+str(1.*len(S.neurons[1].Tspike)/S.timeStop))
    print ('spikes for neuron 2: '+str(len(S.neurons[2].Tspike)) + ', spike rate = '+str(1.*len(S.neurons[2].Tspike)/S.timeStop))
    print ('spikes for neuron 3: '+str(len(S.neurons[3].Tspike)) + ', spike rate = '+str(1.*len(S.neurons[3].Tspike)/S.timeStop))
    print ('********************')


F = pl.figure(figsize=(10,12))
f = F.add_subplot(311)
f.plot(temperatures,NEURON2rate,'.-')
f.set_xlabel('temperature (o C)')
f.set_ylabel('firing rate (spikes / second)')
f.set_title('Neuron 2 (cooling)')

f = F.add_subplot(312)
f.plot(temperatures,NEURON3rate,'.-')
f.set_xlabel('temperature (o C)')
f.set_ylabel('firing rate (spikes / second)')
f.set_title('Neuron 3 (warming)')

f = F.add_subplot(313)
f.plot(temperatures,NEURON2rate,'.-')
f.plot(temperatures,NEURON3rate,'.-')
f.set_xlabel('temperature (o C)')
f.set_ylabel('firing rate (spikes / second)')
f.set_title('combined (neuron 2 and 3)')


pl.show()

'''
gkk




# PLOTTING
time = np.arange(S.steps)*S.dt

F = pl.figure(figsize=(12,8))

f = F.add_subplot(411)
f.plot(time,S.neurons[0].Vrecord)
f.set_xlabel('time (S)')
f.set_ylabel('mV')
f.set_title('Warm-sensitive +')

f = F.add_subplot(412)
f.plot(time,S.neurons[1].Vrecord)
f.set_xlabel('time (S)')
f.set_ylabel('mV')
f.set_title('Warm-sensitive -')

f = F.add_subplot(413)
f.plot(time,S.neurons[2].Vrecord)
f.set_xlabel('time (S)')
f.set_ylabel('mV')
f.set_title("'WARM' --> cooling ")

f = F.add_subplot(414)
f.plot(time,S.neurons[3].Vrecord)
f.set_xlabel('time (S)')
f.set_ylabel('mV')
f.set_title("'COOL' --> warming ")



pl.show()
'''
