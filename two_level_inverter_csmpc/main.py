import matplotlib.pyplot as plt
from pwm.pwm_gen import PWM
from load.load_dyn_cal import Load
from scenario_executor.scenario_exc import sim_executor
from current_reference.current_ref_gen import CurrentReference
from pu_handler.pu_calc import PerUnitBases
from power_current_conv.power_current_handler import RequiredPowerCurrentHandler
import math

# stage cost
def stage_func(i_a, i_a_ref,u_seq_current,u_seq_last):
    return (i_a - i_a_ref)**2 + 0.2*(u_seq_current-u_seq_last)**2

## Simulation parameters
# Control and prediction horizon
cont_horizon = 10

# Power requirements
P_req = 3e3  # Active power in W
Q_req = 0.0    # Reactive power in VAR
S_req = math.sqrt(P_req**2 + Q_req**2)  # Apparent power in VA
V_rms_req = 230.0  # RMS voltage in V
f_req = 50.0  # Reference frequency in Hz

# DC Source Voltage
V_dc = V_rms_req * math.sqrt(2)  # V

# Initial current
i_a_0 = 0.0
# Carrier frequency
carrier_freq = 1e4 # in kHz
# Initial time
t_0 = 0.0          # in sec
# Simulation time
sim_time = 0.02    # in sec
# Overall sampling time
sampling_time = 1/carrier_freq     # in kHz control/plant step (-> 100 samples per PWM period)

# Load parameters
Rl=10.0
Ll=5e-2
back_emf_peak = 100
f_back_emf = f_req

# Create objects
powerCurrentHandler = RequiredPowerCurrentHandler(P_req, Q_req, V_rms_req)
i_ref_peak, phi_ref = powerCurrentHandler.calculateCurrentMagnitudeAndPhase()
pu = PerUnitBases(S_base=S_req,V_rms_base=V_rms_req,Vdc_base=V_dc,f_base=f_req)
# Provide the values of the parameters in Per Unit
pu_vals = pu.convert_2_pu_values(
            f_req, carrier_freq, t_0, sampling_time, sim_time, 
            Rl, Ll, back_emf_peak, V_dc, i_ref_peak, i_a_0)
f_req_pu = pu_vals.f_req
carrier_freq_pu = pu_vals.carrier_freq
t_0_pu = pu_vals.t_0
sampling_time_pu = pu_vals.Ts
sim_time_pu = pu_vals.sim_time
resistance_pu = pu_vals.R
inductance_pu = pu_vals.L
back_emf_peak_pu = pu_vals.e_peak
Vdc_pu = pu_vals.Vdc
i_ref_peak_pu = pu_vals.i_ref_peak
i_a_0_pu = pu_vals.i_a_0

pwm = PWM(carrier_freq_pu, sampling_time_pu, Vdc_pu,per_unit=True)
load = Load(sampling_time_pu, resistance_pu, inductance_pu, back_emf_peak_pu, f_req_pu,per_unit=True)
currentReference = CurrentReference(i_ref_peak_pu, f_req_pu,per_unit=True)
# Initial guess for control
u0 = [0.0]*cont_horizon
# Run simulation
t_sim_pu, v_an_tarj_pu, i_a_traj_pu, i_ref_traj_pu, u_traj, cost_func_val = sim_executor(
    stage_func, pwm, load, currentReference, u0, cont_horizon, t_0_pu, i_a_0_pu, sampling_time_pu, sim_time_pu)
# Return to the scaled values
t_sim = [t*pu.t_base for t in t_sim_pu]  # 
v_an_tarj = [v*pu.Vdc_base for v in v_an_tarj_pu]  # PWM voltage is in p.u. of Vdc_base
i_a_traj = [i*pu.I_base*math.sqrt(2) for i in i_a_traj_pu] # 
i_ref_traj = [i*pu.I_base*math.sqrt(2) for i in i_ref_traj_pu] # 

# Plot results
plt.figure(figsize=(8,7))
plt.subplot(3,1,1)
plt.plot(t_sim, u_traj, label='u')
plt.ylabel('u')
plt.legend()

plt.subplot(3,1,2)
plt.plot(t_sim, v_an_tarj, label='v_avg (per Ts)')
plt.ylabel('V')
plt.legend()

plt.subplot(3,1,3)
plt.plot(t_sim, i_a_traj, label='i_a')
plt.plot(t_sim, i_ref_traj, '--', label='i_ref')
plt.ylabel('A'); plt.xlabel('s')
plt.legend()

plt.tight_layout()
plt.show()