
import math

class Load:
    def __init__(self, sampling_rate, Rl, Ll, back_emf_peak, back_emf_freq, per_unit: bool=False):
        self.Ts = float(sampling_rate)
        self.Rl = float(Rl)
        self.Ll = float(Ll)
        self.V_emf = float(back_emf_peak)
        self.f_emf = float(back_emf_freq)
        self.per_unit = per_unit

    def _bemf(self, t):
        if self.per_unit:
            return self.V_emf * math.cos(t) # OR math.cos(2.0*math.pi*self.f_emf*t) ?!
        else:
            return self.V_emf * math.cos(2.0*math.pi*self.f_emf*t)
        

    def step_euler(self, i_a, v, t, dt):
        # di/dt = ( -R*i + v - e(t) ) / L
        di = (- self.Rl * i_a + v - self._bemf(t)) / self.Ll
        return i_a + dt * di

    def calculateLoadDynamicsSubsteps(self, i_a_0, v_subseq, t_0, dt_sub):
        """
        Advance through a sub-stepped waveform (for PWM).
        """
        i = i_a_0
        t = t_0
        for v in v_subseq:
            i = self.step_euler(i, v, t, dt_sub)
            t += dt_sub
        return [i]  # return final current as a list for consistency
