from scipy.optimize import minimize
import math
import numpy as np
from cost_fun.cost_func_calc import CostFunction
from mpc_contr.mpc_contr_calc import MPCSSolver
from load.load_dyn_cal import Load
from inverter.inverter_behave import Inverter
from current_reference.current_ref_gen import CurrentReference

# === Simulation Parameters ===
i_a_traj = []
i_ref_traj = []
s_traj = []
cost_func_val = []
t_sim = []

# === Run the simulation ===

def sim_executor(load, inverter, mpc, currentReference, s0, cont_horizon = 5, t_0 = 0, i_a_0 = 0, sampling_rate = 1e-4, sim_time = 0.1):

    current_time = t_0
    while current_time < sim_time:
        s_sig, cost_func_val_np = mpc.solveMPC(cont_horizon,inverter,load,currentReference,current_time,i_a_0,s0)
        i_a_next = load.calculateLoadDynamics(i_a_0,inverter.generateOutputVoltage([s_sig.value[0]]),current_time,sampling_rate)[-1]
        i_ref_next = currentReference.generateRefTrajectory(current_time)
        i_a_traj.append(i_a_next)
        i_ref_traj.append(i_ref_next[0])
        t_sim.append(current_time)
        cost_func_val.append(float(cost_func_val_np))
        s_traj.append(s_sig.value[0])
        for i, s in enumerate(s_sig.value):
            if s <= 0.5:
                s0[i] = False
            else:
                s0[i] = True
        i_a_0 = i_a_next
        current_time += sampling_rate

    return s_traj, i_a_traj, i_ref_traj, t_sim, cost_func_val