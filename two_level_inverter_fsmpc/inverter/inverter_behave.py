class Inverter():
    def __init__(self,V_dc=400.0):
        """
        Initializes the Inverter with a given DC voltage.
        """
        self.V_dc = V_dc

    def generateOutputVoltage(self,s_seq, phase_num: int = 1):
        """
        Generates the output voltage trajectory based on the switching signal sequence.
        Parameters:
        - s_seq: Sequence of switching signals (list or array of binary values).
        - phase_num: Number of phases (1 for single-phase, 3 for balanced three-phase).
        Returns:
        - v_inv_trajectory: List of output voltages corresponding to the switching signals.
        """
        if phase_num not in [1, 3]:
            raise ValueError("phase_num must be either 1 (single-phase) or 3 (three-phase).")
        
        if phase_num == 1:
            v_aN_trajectory = []
            for s in s_seq:
                v_aN = self.V_dc * (2*s - 1) / 2.0
                v_aN_trajectory.append(v_aN)
            v_inv_trajectory = v_aN_trajectory
            return v_inv_trajectory
        else: # phase_num == 3
            v_aN_trajectory = []
            v_bN_trajectory = []
            v_cN_trajectory = []
            for s_a, s_b, s_c in s_seq:
                v_aN = self.V_dc * (2*s_a - 1) / 2.0
                v_aN_trajectory.append(v_aN)
                v_bN = self.V_dc * (2*s_b - 1) / 2.0
                v_bN_trajectory.append(v_bN)
                v_cN = self.V_dc * (2*s_c - 1) / 2.0
                v_cN_trajectory.append(v_cN)
            v_inv_trajectory = (v_aN_trajectory, v_bN_trajectory, v_cN_trajectory)
            return v_inv_trajectory