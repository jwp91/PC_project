from scipy.interpolate import interp1d
from scipy.optimize import minimize
from scipy.integrate import odeint
import numpy as np

class PID:
    """Basic PID Controller. 
    Once initialized, the object can be called with a process value to return the controller output."""
    def __init__(self, KC, KI, KD, dt = None, outputLimits = (None, None), ubias = 0.0, setpoint = None):
        self.KC = KC
        self.KI = KI
        self.KD = KD
        self.dt = dt
        self.outputLimits = outputLimits
        self.ubias = ubias
        self.setpoint = setpoint
        # Class variables
        self.ierr = 0                     # Integral error
        self.pvlast = None                # Last process value

    def __call__(self, pv, Iterm:bool = True):
        """Compute the PID output given the process value.
        Inputs:
            pv = float; process value
            Iterm = bool; True if integral action is enabled.
                This can be useful for preventing reset windup when the controller is at a secondary physical limit."""
        s = self
        
        # Confirm necessary parameters are set
        if s.setpoint == None:
            raise ValueError("Setpoint not defined: cannot compute PID output.")
        if s.dt == None:
            raise ValueError("Time increment (dt) not defined: cannot compute integral or derivative errors.")
        if s.pvlast == None:
            s.pvlast = pv
        
        # Calculate errors
        error  = s.setpoint - pv
        s.ierr = s.ierr + s.KI*error*s.dt
        s.dpv  = (pv-s.pvlast) / s.dt
        
        # Calculate PID output
        P = s.KC*error
        if Iterm:
            I = s.ierr
        else:
            I = 0
        D = -s.KD*s.dpv
        op = s.ubias + P + I + D
        # print(f"Setpoint = {s.setpoint}, PV = {pv}, OP = {op}, P = {P}, I = {I}, D = {D}") #DEGBUGGING

        # implement anti-reset windup
        oplo = min(s.outputLimits)
        ophi = max(s.outputLimits)
        if oplo != None and Iterm:
            if op < oplo:
                s.ierr -= s.KI*error*s.dt
                op = max(oplo, op)
        if ophi != None and Iterm:
            if op > ophi:
                s.ierr -= s.KI*error*s.dt
                op = min(ophi, op)

        s.pvlast = pv
        # return the controller output and PID terms
        return [op,P,I,D]
    
class FOPDTopt:
    def __init__(self, ts, data, ig, uData):
        """
        Initialize the FOPDTopt object.

        Parameters
        ----------
        ts : array of time values
        data : array of process variable values
        ig : array containing initial guess for [Kp, thetap, taup]
        uData : Data array for u (manipulated variable)

        Attributes
        ----------
        yOpt : array of data solved by integrating the FOPDT model with optimized parameters
        yInit: array of data solved by integrating the FOPDT model with initial guess as parameters
        Kp, thetap, taup : optimized parameters
        """
        self.ts = ts
        self.npts = len(self.ts)-1
        self.data = data
        self.ig = ig
        self.uData = uData
        self.uFunc = interp1d(self.ts, self.uData, bounds_error = False, fill_value = (self.uData[0],self.uData[-1]))
        self.yOpt, self.Kp, self.thetap, self.taup = self.compute()
        self.yInit, extra1, extra2, extras2 = self.compute(optimize = False)

    def compute(self, optimize = True):
        if optimize:
            print("optimizing...")
            params = minimize(self.goal, self.ig).x
        else:
            print("Using ig as params")
            params = self.ig
        return self.solveFOPDT(params), *params

    
    def fopdt(self, y, t, params):
        Kp, thetap, taup = params
        if t-thetap < 0:
            dydt = (-(y-self.data[0]) + Kp*(self.uFunc(0) - self.uData[0]))/taup
        else:
            dydt = (-(y-self.data[0]) + Kp*(self.uFunc(t-thetap) - self.uData[0]))/taup
        return dydt


    def solveFOPDT(self, params):
        yOpt = np.ones(self.npts+1)*self.data[0]
        for i in range(self.npts):
            # Integrate the model
            y = odeint(self.fopdt,yOpt[i],[self.ts[i],self.ts[i+1]], args = (params,))
            # Record the result
            yOpt[i+1] = y[-1][0]
        return yOpt

    def goal(self, params): 
        yOpt = self.solveFOPDT(params)
        sumsq = np.sum((yOpt-self.data)**2)
        return sumsq
    
    def IMC(self, mode = 'aggressive'):
        """Computes the IMC parameters for PID tuning given a mode
        Inputs:
            Mode = string; 'aggressive', 'moderate', or 'conservative'
        Ouputs:
            Kc, tauc, taui, taud
        """
        scalar = {'aggressive': 1.0, 'moderate': 10., 'conservative': 100.}[mode]
        tauc = max(0.1*self.taup, 0.8*self.thetap)*scalar
        Kc = 1/self.Kp*(self.taup + 0.5*self.thetap)/(tauc + 0.5*self.thetap)
        taui = self.taup + 0.5*self.thetap
        taud = self.taup*self.thetap/(2*self.taup + self.thetap)
        return Kc, tauc, taui, taud