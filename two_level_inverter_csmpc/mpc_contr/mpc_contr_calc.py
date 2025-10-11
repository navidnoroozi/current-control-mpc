
from scipy.optimize import minimize

class MPCSSolver:
    def __init__(self, cost_func, cont_horizon=10):
        self.cost_func = cost_func
        self.cont_horizon = cont_horizon

    def solveMPC(self, pwm, load, currentReference, t_0, i_a_0, u0=None):
        if u0 is None:
            u0 = [0.0] * self.cont_horizon

        def objective(u_seq):
            return self.cost_func.calculateCostFunc(i_a_0, t_0, u0, self.cont_horizon, u_seq, pwm, load, currentReference)

        bounds = [(-1, 1)] * self.cont_horizon
        res = minimize(objective, u0, method='trust-constr', bounds=bounds)

        if not res.success:
            return u0, objective(u0)
        return res.x, res.fun