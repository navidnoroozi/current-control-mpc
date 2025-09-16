import math

class PWM:
    def __init__(self, carrier_freq=1e6, Ts=1e-5, Vdc=520.0):
        """
        carrier_freq : float
            Frequency of triangular carrier [Hz].
        Ts : float
            Simulation sampling time [s].
        Vdc : float
            DC bus voltage [V].
        """
        self.fc = carrier_freq
        self.Ts = Ts
        self.Vdc = Vdc
        
        # Triangular carrier generator parameters
        self.Tc = 1.0 / self.fc
        self.carrier = 0.0
        self.carrier_dir = 1  # +1 rising, -1 falling

    def find_k(self,t_0):
        """
        Find the base interval index k for time t given carrier period Tc.

        Parameters
        ----------
        t : float
            Current time.
        Tc : float
            Carrier period.

        Returns
        -------
        k : int
            Base interval index.
        half : str
            'first_half' if t is in the first half of the interval, 'second_half' if in the second half.
        """
        if self.Tc <= 0:
            raise ValueError("Tc must be positive")
        k = math.floor(t_0 / self.Tc)  # base interval index
        lower = k * self.Tc
        midpoint = (k + 0.5) * self.Tc
        upper = (k + 1) * self.Tc
    
        if lower <= t_0 <= midpoint:
            return k, "first_half"
        elif midpoint < t_0 < upper:
            return k, "second_half"
        else:
            return None, "out_of_range"

    def generateOneSampleGatingSignal(self, duty_cyc,t_0):
        """
        Perform one simulation step of PWM.

        duty_cyc : float
            Control input in [-1, 1] from MPC.

        Returns
        -------
        v_out : float
            Output inverter voltage applied to the load.
        g1, g2 : int
            Gating signals for the two switches of the half-bridge leg.
        """
        # Update triangular carrier (sawtooth/triangular)
        k, half = self.find_k(t_0)
        if half == "first_half":
            self.carrier = (t_0 - k * self.Tc) * (2.0 / self.Tc) #- 1.0
        elif half == "second_half":
            self.carrier = - (t_0 - (k + 1) * self.Tc) * (2.0 / self.Tc) #+ 1.0
        else:
            raise ValueError("Time t_0 is out of range for carrier generation.")

        # Compare duty cycle with carrier to generate gating signal
        if (duty_cyc+1)/2 >= self.carrier:
            g1 = 1
            g2 = 0
            v_out = self.Vdc / 2.0   # Inverter output voltage
        else:
            g1 = 0
            g2 = 1
            v_out = -self.Vdc / 2.0  # Inverter output voltage

        # g1 = 1 if duty_cyc >= self.carrier else 0
        # g2 = 1 - g1  # Complementary switch

        # # Inverter output voltage
        # v_out = (g1 - g2) * (self.Vdc / 2)

        return v_out, g1, g2
    
    def generateGatingSignals(self,duty_cyc_seq,t_0):
        """
        Generate gating signals for a sequence of duty cycles.

        duty_cycle_sequence : array-like
            Sequence of control inputs in [-1, 1] from MPC.

        Returns
        -------
        v_out_seq : list of float
            Sequence of output inverter voltages applied to the load.
        g1_seq, g2_seq : list of int
            Sequences of gating signals for the two switches of the half-bridge leg.
        """
        v_out_seq = []
        g1_seq = []
        g2_seq = []
        t_current = t_0
        for duty_cyc in duty_cyc_seq:
            v_out, g1, g2 = self.generateOneSampleGatingSignal(duty_cyc,t_current)
            v_out_seq.append(v_out)
            g1_seq.append(g1)
            g2_seq.append(g2)
            t_current += self.Ts

        return v_out_seq, g1_seq, g2_seq