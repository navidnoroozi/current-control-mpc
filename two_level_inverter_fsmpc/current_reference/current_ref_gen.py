import numpy as np
import math

class CurrentReference():
    def __init__(self,sampling_rate,cont_horizon,I_ref_peak=5.0,phi_ref=0.0,f_ref=50.0):
        """
        Initializes the Current Reference generator.

        Parameters:
        - sampling_rate: Time step for simulation.
        - cont_horizon: Control horizon.
        - I_ref_peak: Peak value of the reference current.
        - phi_ref: Phase angle of the reference current in radians.
        - f_ref: Frequency of the reference current in Hz.
        """
        self.sampling_rate = sampling_rate
        self.cont_horizon = cont_horizon
        self.I_ref_peak = I_ref_peak
        self.phi_ref = phi_ref
        self.f_ref = f_ref

    def generateRefTrajectory(self, phase_num: int = 1, t_0: float = 0.0) -> tuple:
        """
        Generates the reference current trajectory over the control horizon.

        Parameters:
        - phase_num: Number of phases (1 for single-phase, 3 for balanced three-phase).
        - t_0: Initial time.

        Returns:
        - Reference current trajectory in a single phase i_a or balanced three-phaseas [i_a,i_b,i_c] a list.
        Note:
        The returned current is the instantaneous value, not RMS.
        """
        if phase_num not in [1, 3]:
            raise ValueError("phase_num must be either 1 (single-phase) or 3 (three-phase).")
        if phase_num == 1:
            i_a_ref_traj = []
            for i in range(self.cont_horizon):
                t = t_0 + i * self.sampling_rate
                i_a_ref = self.I_ref_peak * math.cos(2 * math.pi * self.f_ref * t - self.phi_ref)
                i_a_ref_traj.append(i_a_ref)
            return (np.array(i_a_ref_traj),)
        else: # phase_num == 3
            i_a_ref_traj = []
            i_b_ref_traj = []
            i_c_ref_traj = []
            for i in range(self.cont_horizon):
                t = t_0 + i * self.sampling_rate
                i_a_ref = self.I_ref_peak * math.cos(2 * math.pi * self.f_ref * t - self.phi_ref)
                i_b_ref = self.I_ref_peak * math.cos(2 * math.pi * self.f_ref * t - self.phi_ref - 2*math.pi/3)
                i_c_ref = self.I_ref_peak * math.cos(2 * math.pi * self.f_ref * t - self.phi_ref + 2*math.pi/3)
                i_a_ref_traj.append(i_a_ref)
                i_b_ref_traj.append(i_b_ref)
                i_c_ref_traj.append(i_c_ref)
            return (np.array(i_a_ref_traj), np.array(i_b_ref_traj), np.array(i_c_ref_traj))