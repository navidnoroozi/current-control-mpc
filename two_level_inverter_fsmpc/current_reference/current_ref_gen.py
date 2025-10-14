import math
import numpy as np

class CurrentReference:
    def __init__(self, sampling_rate, cont_horizon, I_ref_peak=5.0, phi_ref=0.0, f_ref=50.0):
        self.sampling_rate = float(sampling_rate)
        self.cont_horizon = int(cont_horizon)
        self.I_ref_peak = float(I_ref_peak)
        self.phi_ref = float(phi_ref)
        self.f_ref = float(f_ref)

    # --- single-phase (legacy) ---
    def generateRefTrajectory(self, t_0):
        Ts = self.sampling_rate
        N = self.cont_horizon
        w = 2.0 * math.pi * self.f_ref
        vals = [ self.I_ref_peak * math.cos(w*(t_0 + k*Ts) - self.phi_ref) for k in range(N) ]
        return np.array(vals, dtype=float)

    # --- balanced three-phase ---
    def generateThreePhaseRefs(self, t_0):
        Ts = self.sampling_rate
        N = self.cont_horizon
        w = 2.0 * math.pi * self.f_ref
        ia, ib, ic = [], [], []
        phi_a = self.phi_ref
        phi_b = self.phi_ref - 2.0*math.pi/3.0
        phi_c = self.phi_ref + 2.0*math.pi/3.0
        for k in range(N):
            t = t_0 + k*Ts
            ia.append(self.I_ref_peak * math.cos(w*t + phi_a))
            ib.append(self.I_ref_peak * math.cos(w*t + phi_b))
            ic.append(self.I_ref_peak * math.cos(w*t + phi_c))
        return np.array(ia), np.array(ib), np.array(ic)