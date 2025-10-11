
from cost_fun.cost_func_calc import CostFunction
from mpc_contr.mpc_contr_calc import MPCSSolver

def sim_executor(stage_func,pwm,load,currentReference,u0, cont_horizon, t_0, i_a_0,
                 sampling_rate, sim_time):
    """
    Execute a simulation with MPC controlling the inverter-load system.
    The MPC uses the averaged model for prediction (no switching inside the MPC).
    The plant simulation uses switching synthesis and sub-stepping to capture dynamics.

    Parameters:
    - stage_func: function handle for stage cost (i_a, i_a_ref) -> cost
    - cont_horizon: control and prediction horizon (number of steps)
    - V_dc: DC link voltage
    - t_0: initial time
    - u0: initial guess for control sequence (list of length cont_horizon)
    - i_a_0: initial current
    - carrier_freq: PWM carrier frequency
    - sampling_rate: control and plant step size (Ts)
    - sim_time: total simulation time
    - f_ref: reference frequency for current reference
    - pwm: PWM object
    - load: Load object
    - currentReference: CurrentReference object

    Returns:
    - t_sim: list of time points
    - v_an_tarj: list of averaged voltage trajectory (per Ts)
    - i_a_traj: list of current trajectory
    - i_ref_traj: list of reference current trajectory
    - u_traj: list of control inputs applied
    - cost_func_val: list of cost function values at each step
    """
    cost_func = CostFunction(stage_func)
    solver = MPCSSolver(cost_func, cont_horizon=cont_horizon)

    v_an_tarj = []
    i_a_traj = []
    i_ref_traj = []
    u_traj = []
    t_sim = []
    cost_func_val = []

    current_time = t_0
    u_prev = u0
    
    steps = int(sim_time / sampling_rate)
    for _ in range(steps):
        # Solve MPC (averaged model inside)
        u_seq_opt, J = solver.solveMPC(pwm, load, currentReference, t_0=current_time, i_a_0=i_a_0, u0=u_prev)
        u_k = float(u_seq_opt[0])

        # Apply to the load with switching synthesis
        # synthesize high-frequency switching over one step and sub-integrate
        v_sub, dt_sub = pwm.synthesize_over_interval(u_k, current_time, Ts=sampling_rate)
        i_next_list = load.calculateLoadDynamicsSubsteps(i_a_0, v_sub, current_time, dt_sub)
        v_avg = sum(v_sub)/len(v_sub)
        v_an_next = [v_avg]  # store averaged voltage for plotting

        i_next = i_next_list[-1]
        i_ref = currentReference.generateRefTrajectory(current_time)[0]

        # Log
        cost_func_val.append(float(J))
        v_an_tarj.append(v_an_next[0])
        i_a_traj.append(i_next)
        u_traj.append(u_k)
        t_sim.append(current_time)
        i_ref_traj.append(i_ref)

        # Prepare next step
        i_a_0 = i_next
        u_prev = u_seq_opt
        current_time += sampling_rate

    return t_sim, v_an_tarj, i_a_traj, i_ref_traj, u_traj, cost_func_val
