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