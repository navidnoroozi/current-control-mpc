from scipy.optimize import minimize
import math
import numpy as np
from cost_fun.cost_func_calc import CostFunction
from mpc_contr.mpc_contr_calc import MPCSSolver
from load.load_dyn_cal import Load
from inverter.inverter_behave import Inverter
from current_reference.current_ref_gen import CurrentReference
from pwm.pwm_gen import PWM

v_an_tarj = []
i_a_traj = []
i_ref_traj = []
s_traj = []
t_sim = []
cost_func_val = []
def sim_executor(stage_func, cont_horizon, V_dc, t_0, s0, i_a_0, carrier_freq, sampling_rate, mpc_method, sim_time, with_pwm=False):
    current_time = t_0
    while current_time < sim_time:
        # === Create subsystem instances and solve the switching signal ===
        currentReference = CurrentReference(sampling_rate,cont_horizon,I_ref_peak=5.0,f_ref=50*2*math.pi)
        load = Load(sampling_rate, resistance=1.0, inductance=2e-3,back_emf_peak=0.0,back_emf_freq=50.0*2.0*math.pi)
        pwm = PWM(carrier_freq, Ts=sampling_rate, Vdc=V_dc)
        inverter = Inverter(V_dc)
        cost_func = CostFunction(stage_func)
        mpc = MPCSSolver(cost_func,cont_horizon)
        if with_pwm:
            s_sig_pred_array,cost_func_val_np = mpc.solveMPC(cont_horizon,pwm,load,currentReference,current_time,i_a_0,s0,mpc_method,with_pwm=True)
        else:
            s_sig_pred_array,cost_func_val_np = mpc.solveMPC(cont_horizon,inverter,load,currentReference,current_time,i_a_0,s0,mpc_method,with_pwm=False)
        s_sig_pred = s_sig_pred_array.tolist()        
        # print("Next switching value:", s_sig_pred[0])
        # === Calculate the state trajectory using the dynamics ===
        if with_pwm:
            v_an_next, _, _ = pwm.generateGatingSignals([s_sig_pred[0]],current_time)
        else:
            v_an_next = inverter.generateOutputVoltage([s_sig_pred[0]],mpc_method)
        i_a_next = load.calculateLoadDynamics(i_a_0,[v_an_next[0]],current_time)
        i_ref_next = currentReference.generateRefTrajectory(current_time)
        # === Store the results ===
        cost_func_val.append(float(cost_func_val_np))
        v_an_tarj.append(v_an_next[0])
        i_a_traj.append(i_a_next[0])
        s_traj.append(s_sig_pred[0])
        t_sim.append(current_time)
        i_ref_traj.append(i_ref_next[0])
        # === Update the initial conditions for the next iteration ===
        i_a_0 = i_a_next[-1]
        s0 = s_sig_pred
        current_time += sampling_rate
    return t_sim, v_an_tarj, i_a_traj, i_ref_traj, s_traj, cost_func_val