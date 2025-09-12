from scipy.optimize import minimize
import cvxpy as cp

class MPCSSolver:
    def __init__(self,cont_horizon):
        """
        Initializes the MPC Solver.

        Parameters:
        - cost_func: Instance of CostFunction to evaluate the cost.
        - cont_horizon: Control horizon.
        """
        self.cont_horizon = cont_horizon

    def solveMPC(self,cont_horizon,inverter,load,currentReference,current_time,i_a_0,s0):
        """
        Solves the MPC optimization problem to find the optimal switching signal sequence.

        Parameters:
        - cont_horizon: Control horizon.
        - inverter: Instance of Inverter class.
        - load: Instance of Load class.
        - currentReference: Instance of CurrentReference class.
        - current_time: Current time.
        - i_a_0: Initial load current.
        - s0: Initial guess for switching signal sequence.
        - with_pwm: Boolean indicating if PWM is used.

        Returns:
        - Optimal switching signal sequence as a numpy array.
        - Value of the cost function at the optimum.
        """
        s_seq = cp.Variable(cont_horizon, boolean=True)  # Binary decision variables
        # if s_seq.value is None:
        s_seq.value = s0
        
        # constraints = [s_seq >= -1, s_seq <= 1]
        t_0 = current_time
        v_an_tarj = inverter.generateOutputVoltage(s_seq)
        i_a_traj = load.calculateLoadDynamics(i_a_0,v_an_tarj,t_0)
        i_a_ref_traj = currentReference.generateRefTrajectory(t_0)
        i_a_traj = cp.hstack(i_a_traj)
        cost_func = cp.sum_squares(i_a_ref_traj - i_a_traj) 
        
        prob = cp.Problem(cp.Minimize(cost_func)) # , constraints
        
        prob.solve(solver=cp.ECOS_BB)  # small problems only
        
        return s_seq, prob.value