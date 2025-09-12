from scipy.optimize import minimize
import cvxpy as cp


class MPCSSolver():
    def __init__(self, cost_func,cont_horizon=10):
        """
        Initializes the MPC solver.
        """
        self.cost_func = cost_func
        self.cont_horizon = cont_horizon

    def solveMPC(self,cont_horizon,pwm_or_inverter,load,currentReference,t_0=None,i_a_0=None,s0=None,mpc_method=None,with_pwm=False):
        """
        Solves the MPC problem.

        Parameters:
        - x0: Initial condition for the state variable x.
        - u0: Initial guess for the control sequence u.

        Returns:
        - The final value, iteration count, and cost function value.
        """
        if i_a_0 is None:
            i_a_0 = 0.0
        if t_0 is None:
            t_0 = 0.0
        if mpc_method is None:
            assert False, "Please specify the MPC method: 'FSMPC' or 'PWM'"
        if mpc_method not in ['FSMPC','PWM']:
            assert False, "Please specify the MPC method: 'FSMPC' or 'PWM'"
        if mpc_method == 'PWM':
            if s0 is None:
                s0 = [0.0] * cont_horizon
        if mpc_method == 'FSMPC':
            # Binary decision variables
            s_seq = cp.Variable(cont_horizon) # , boolean=True
            if s0 is None:
                s0 = [False] * cont_horizon
            s_seq.value = s0  # Initial guess

        # Define the objective function to minimize
        def calculateCbjective(s_seq):
            if with_pwm:
                return self.cost_func.calculateCostFunc(i_a_0,t_0,cont_horizon,s_seq,pwm_or_inverter,load,currentReference,mpc_method,with_pwm=True)
            else:
                if mpc_method == 'FSMPC':
                    return self.cost_func.calculateCostFunc(i_a_0,t_0,cont_horizon,s_seq.value,pwm_or_inverter,load,currentReference,mpc_method,with_pwm=False)
                elif mpc_method == 'PWM':
                    return self.cost_func.calculateCostFunc(i_a_0,t_0,cont_horizon,s_seq,pwm_or_inverter,load,currentReference,mpc_method,with_pwm=False)

        # The objective function is the cost function to be minimized
        if mpc_method == 'FSMPC':
            # Define the objective function based on the method
            objective = cp.Minimize(calculateCbjective(s_seq))
            # Constraints (just binary condition, already set)
            constraints = [
            s_seq >= -1,
            s_seq <= 1
            ]
            # Call the optimizer        
            problem = cp.Problem(objective, constraints)  #, constraints
            # Solve with ECOS_BB (default integer solver), other solvers: CBC, GLPK_MI, SCIP, GUROBI
            problem.solve()  #, warm_start=True
            # print("Status:", problem.status)
            # print("Optimal value:", problem.value)
            # print("Optimal s_seq:", s_seq.value)
            return s_seq.value,problem.value
        elif mpc_method == 'PWM':
            # Define the objective function based on the method
            objective = lambda s_seq: calculateCbjective(s_seq)
            # Bounds for each variable: (lower, upper)
            bounds = [(-1, 1)] * self.cont_horizon  # applies to s1,s2,...,sN
            # Call the optimizer
            result = minimize(objective, s0, method='trust-constr',bounds=bounds) # possible solvers: 'L-BFGS-B', 'SLSQP', 'trust-constr'
            # Print result
            # print("Success:", result.success)
            # print("Optimal value:", result.fun)
            # print("Optimal s_seq:", result.x)
            return result.x,result.fun