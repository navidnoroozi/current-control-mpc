import math

class Load():
    def __init__(self,R=10.0,L=0.005,V_backemf=0.0,f_backemf=50.0):
        """
        Initializes the Load with given resistance and inductance.

        Parameters:
        - R: Resistance in ohms.
        - L: Inductance in henrys.
        - V_backemf: Back electromotive force.
        - f_backemf: Frequency of back EMF.
        """
        self.R = R
        self.L = L
        self.V_backemf = V_backemf
        self.f_backemf = f_backemf

    def calculateLoadDynamics(self, i_a_0, v_inv_tarj, phase_num: int = 1, t_0: float = 0.0, sampling_rate: float = 1e-4) -> tuple:
        """
        Calculates the load current trajectory based on the applied voltage.

        Parameters:
        - i_a_0: Initial load current.
        - v_inv_tarj: Applied voltage trajectory.
        - phase_num: Number of phases (1 for single-phase, 3 for balanced three-phase).
        - t_0: Initial time.
        - sampling_rate: Time step for simulation.

        Returns:
        - Load current trajectory as a list of float in tuple.
        """
        if phase_num not in [1, 3]:
            raise ValueError("phase_num must be either 1 (single-phase) or 3 (three-phase).")
        
        if phase_num == 1:
            i_a_traj = []
            t = t_0
            i_a = i_a_0
            for v_an in v_inv_tarj:
                v_backemf = self.V_backemf * math.sin(2 * math.pi * self.f_backemf * t)
                di_dt = (v_an - v_backemf - self.R * i_a) / self.L
                i_a_next = i_a + di_dt * sampling_rate
                i_a_traj.append(i_a_next)
                i_a = i_a_next
                t += sampling_rate
            return (i_a_traj,)
        else:  # phase_num == 3
            i_a_traj = []
            i_b_traj = []
            i_c_traj = []
            t = t_0
            i_a, i_b, i_c = i_a_0
            for v_an, v_bn, v_cn in v_inv_tarj:
                v_backemf_a = self.V_backemf * math.sin(2 * math.pi * self.f_backemf * t)
                v_backemf_b = self.V_backemf * math.sin(2 * math.pi * self.f_backemf * t - 2*math.pi/3)
                v_backemf_c = self.V_backemf * math.sin(2 * math.pi * self.f_backemf * t + 2*math.pi/3)
                
                di_a_dt = (v_an - v_backemf_a - self.R * i_a) / self.L
                di_b_dt = (v_bn - v_backemf_b - self.R * i_b) / self.L
                di_c_dt = (v_cn - v_backemf_c - self.R * i_c) / self.L
                
                i_a_next = i_a + di_a_dt * sampling_rate
                i_b_next = i_b + di_b_dt * sampling_rate
                i_c_next = i_c + di_c_dt * sampling_rate
                
                i_a_traj.append(i_a_next)
                i_b_traj.append(i_b_next)
                i_c_traj.append(i_c_next)
                
                i_a, i_b, i_c = i_a_next, i_b_next, i_c_next
                t += sampling_rate
            return (i_a_traj, i_b_traj, i_c_traj)