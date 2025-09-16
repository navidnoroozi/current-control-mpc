import matplotlib.pyplot as plt
from scenario_executor.scenario_exc import sim_executor
from itertools import islice, cycle
import numpy as np

# === Define the stage function ===
def stage_func(i_a,i_a_ref):
    return (i_a-i_a_ref)**2

# Control horizon
cont_horizon = 10

# DC Source Voltage
V_dc=100.0

# Initial time
t_0 = 0.0

# Simulation time
sim_time = 10e-4 # 50e-6 # 50e-3 # 

# Initial state
i_a_0 = 5.0

# Sampling rate
sampling_rate = 1e-4 #25e-6

# Carrier frequency for PWM
carrier_freq = 1e3  # 1 kHz

# Use of PWM
with_pwm = False  # True or False

# MPC method
mpc_method = 'PWM'  # 'FSMPC' or 'PWM'

# Initial switching state
if mpc_method == 'FSMPC':
    s0 = np.random.uniform(-1, 1, 10) # list(islice(cycle([True, False]), cont_horizon))  # Initial guess for switching signal sequence
elif mpc_method == 'PWM':
    s0 = [0.0]*cont_horizon  # Initial guess for switching signal sequence
else:
    assert False, "Please specify the MPC method: 'FSMPC' or 'PWM'"

# === Run the simulation ===
t_sim, v_an_tarj, i_a_traj, i_ref_traj, s_traj, cost_func_val = sim_executor(stage_func, cont_horizon, V_dc, t_0, s0, i_a_0, carrier_freq, sampling_rate, mpc_method, sim_time, with_pwm)

# === Plot the results ===
plt.figure()
plt.subplot(3,1,1)
plt.step(t_sim, s_traj, label='Switching Signal s')
plt.title('Switching Signal')
plt.xlabel('Time (s)')
plt.ylabel('s')
plt.grid()
plt.legend()
plt.xlim([t_0,sim_time])
plt.subplot(3,1,2)
plt.plot(t_sim, i_a_traj, label='Load Current i_a')
plt.plot(t_sim, i_ref_traj, label='Reference Current i_a_ref', linestyle='--')
plt.title('Load Current')
plt.xlabel('Time (s)')
plt.ylabel('i_a (A)')
plt.grid()
plt.legend()
plt.xlim([t_0,sim_time])
plt.subplot(3,1,3)
plt.plot(t_sim, v_an_tarj, label='Output Voltage v_an')
plt.title('Output Voltage')
plt.xlabel('Time (s)')
plt.ylabel('v_an (V)')
plt.grid()
plt.legend()
plt.tight_layout()
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