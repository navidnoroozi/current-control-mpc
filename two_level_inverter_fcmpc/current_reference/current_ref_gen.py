import numpy as np
import math



class CurrentReference():
    def __init__(self, sampling_rate,pred_horizon,I_ref_peak=10,f_ref=50*2*math.pi):
        """
        Initializes the Plant with a system dynamics calculator.

        Parameters:
        - sys_dyn_cal: An instance of SysDynCal that calculates system dynamics.
        """
        self.I_ref_peak = I_ref_peak
        self.f_ref = f_ref
        self.pre_horizon = pred_horizon
        self.Ts = sampling_rate

    def generateRefTrajectory(self,t_0):
        """
        Calculates the next state of the plant based on the current state and control input.

        Parameters:
        - state: Current state of the plant.
        - control_input: Control input to the plant.

        Returns:
        - Next state of the plant.
        """
        i_a_ref_trajectory = []
        t_current = t_0
        for _ in range(self.pre_horizon):
            i_a_ref = self.I_ref_peak * math.sin(self.f_ref*t_current)
            i_a_ref_trajectory.append(i_a_ref)
            t_current += self.Ts
        return i_a_ref_trajectory