
class Load():
    def __init__(self,R=10.0,L=0.005):
        """
        Initializes the Load with given resistance and inductance.

        Parameters:
        - R: Resistance in ohms.
        - L: Inductance in henrys.
        """
        self.R = R
        self.L = L

    def calculateLoadDynamics(self,i_a_0,v_an_tarj,t_0,sampling_rate=1e-4):
        """
        Calculates the load current trajectory based on the applied voltage.

        Parameters:
        - i_a_0: Initial load current.
        - v_an_tarj: Applied voltage trajectory.
        - t_0: Initial time.
        - sampling_rate: Time step for simulation.

        Returns:
        - Load current trajectory as a list.
        """
        i_a_traj = []
        t = t_0
        for v_an in v_an_tarj:
            di_dt = (v_an - self.R * i_a_0) / self.L
            i_a_next = i_a_0 + di_dt * sampling_rate
            i_a_traj.append(i_a_next)
            i_a_0 = i_a_next
            t += sampling_rate
        return i_a_traj