class Inverter():
    def __init__(self,V_dc):
        """
        Initializes the Plant with a system dynamics calculator.

        Parameters:
        - sys_dyn_cal: An instance of SysDynCal that calculates system dynamics.
        """
        self.V_dc = V_dc

    def generateOutputVoltage(self,s_seq):
        """
        Calculates the next state of the plant based on the current state and control input.

        Parameters:
        - state: Current state of the plant.
        - control_input: Control input to the plant.

        Returns:
        - Next state of the plant.
        """
        v_aN_trajectory = []
        for s in s_seq:
            v_aN = self.V_dc * (2*s - 1) / 2.0
            v_aN_trajectory.append(v_aN)
        
        return v_aN_trajectory