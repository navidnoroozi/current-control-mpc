import numpy as np
import cvxpy as cp

class Inverter():
    def __init__(self,V_dc):
        """
        Initializes the Plant with a system dynamics calculator.

        Parameters:
        - sys_dyn_cal: An instance of SysDynCal that calculates system dynamics.
        """
        self.V_dc = V_dc

    def generateOutputVoltage(self,s_seq,mpc_method):
        """
        Calculates the next state of the plant based on the current state and control input.

        Parameters:
        - state: Current state of the plant.
        - control_input: Control input to the plant.

        Returns:
        - Next state of the plant.
        """
        v_aN_trajectory = []
        if s_seq is None:
            s_seq = np.random.uniform(-1, 1, 10)
        for s in s_seq:
            if mpc_method == 'PWM':
                v_aN = self.V_dc*s
            elif mpc_method == 'FSMPC':
                if s > 0.0:
                    v_aN = self.V_dc / 2.0
                else:
                    v_aN = -self.V_dc / 2.0
            else:
                assert False, "Please specify the MPC method: 'FSMPC' or 'PWM'"
            v_aN_trajectory.append(v_aN)
        return v_aN_trajectory