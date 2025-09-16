import numpy as np
import math


class Load():
    def __init__(self, sampling_rate=1e-3, resistance=10.0, inductance=10e-3,back_emf_peak=100.0,back_emf_freq=50.0*2.0*math.pi):
        """
        Initializes the Plant with a system dynamics calculator.

        Parameters:
        - sys_dyn_cal: An instance of SysDynCal that calculates system dynamics.
        """
        self.Ts = sampling_rate
        self.Rl = resistance
        self.Ll = inductance
        self.V_emf = back_emf_peak
        self.f_emf = back_emf_freq

    def calculateLoadDynamics(self,i_a_0,v_an_tarj,t_0):
        """
        Calculates the next state of the plant based on the current state and control input.

        Parameters:
        - state: Current state of the plant.
        - control_input: Control input to the plant.

        Returns:
        - Next state of the plant.
        """
        i_a_trajectory = [i_a_0]
        i_a = i_a_0
        t_current = t_0
        for v in v_an_tarj:
            i_a = i_a + self.Ts * (- self.Rl * i_a + v - self.V_emf * math.cos(self.f_emf*t_current)) / self.Ll
            i_a_trajectory.append(i_a)
            t_current += self.Ts
        return i_a_trajectory