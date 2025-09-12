import matplotlib.pyplot as plt
from itertools import islice, cycle
import numpy as np
import math
from scipy.optimize import minimize
import cvxpy as cp
from cost_fun.cost_func_calc import CostFunction
from mpc_contr.mpc_contr_calc import MPCSSolver
from load.load_dyn_cal import Load
from inverter.inverter_behave import Inverter
from current_reference.current_ref_gen import CurrentReference
from scenario_executor.scenario_exc import sim_executor


# Control horizon
cont_horizon = 5

# DC Source Voltage
V_dc=100.0

# Initial time
t_0 = 0.0

# Simulation time
sim_time = 40e-4 # 50e-6 #

# Initial state
i_a_0 = 5.0

# Sampling rate
sampling_rate = 25e-6

inverter = Inverter(V_dc)
load = Load(R=1.0,L=2e-3)
currentReference = CurrentReference(sampling_rate,cont_horizon,I_ref_peak=5.0,f_ref=50*2*math.pi)
mpc = MPCSSolver(cont_horizon)


# Initial switching state
s0 = list(islice(cycle([True, False]), cont_horizon))  # Initial guess for switching signal sequence

s_traj, i_a_traj, i_ref_traj, t_sim, cost_func_val = sim_executor(load, inverter, mpc, currentReference, s0, cont_horizon, t_0, i_a_0, sampling_rate, sim_time)

# === Plot the results ===
plt.figure()
plt.subplot(2,1,1)
plt.step(t_sim, s_traj, label='Switching Signal s')
plt.title('Switching Signal')
plt.xlabel('Time (s)')
plt.ylabel('s')
plt.grid()
plt.legend()
plt.xlim([t_0,sim_time])
plt.subplot(2,1,2)
plt.plot(t_sim, i_a_traj, label='Load Current i_a')
plt.plot(t_sim, i_ref_traj, label='Reference Current i_a_ref', linestyle='--')
plt.title('Load Current')
plt.xlabel('Time (s)')
plt.ylabel('i_a (A)')
plt.grid()
plt.legend()
plt.xlim([t_0,sim_time])


plt.figure()
plt.plot(t_sim, cost_func_val, label='Cost Function Value')
plt.title('Cost Function Value Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Cost Function Value')
plt.grid()
plt.legend()
plt.tight_layout()
plt.xlim([t_0,sim_time])

plt.show()