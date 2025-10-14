import math, os, json
from itertools import islice, cycle
from power_current_conv.power_current_handler import RequiredPowerCurrentHandler
from mpc_contr.mpc_contr_calc import MPCSSolver, clarke_power_invariant
from load.load_dyn_cal import Load
from inverter.inverter_behave import Inverter
from current_reference.current_ref_gen import CurrentReference
from scenario_executor.scenario_exc import sim_executor
import plotly.graph_objects as go
from plotly.subplots import make_subplots

def simPowerConvControlSyst(P_req=3e3, Q_req=0.0, V_rms_req=230.0, cont_horizon=5,
                            t_0=0.0, i_a_0=5.0, sampling_rate=25e-6, sim_time=4e-2,
                            R=10.0, L=10e-3, V_backemf=100.0, f_ref=50.0,
                            phase_num=1, i_b_0=5.0*math.cos(2*math.pi/3), i_c_0=5.0*math.cos(-2*math.pi/3),
                            use_gurobi=True):
    """Run MPC simulation and generate plots.
    Returns (fig1, fig2) plotly Figures.
    """
    V_dc = V_rms_req*math.sqrt(2)*2
    inverter = Inverter(V_dc)
    load = Load(R,L,V_backemf,f_backemf=f_ref)
    PCH = RequiredPowerCurrentHandler(P_req,Q_req,V_rms_req)
    I_ref_peak, phi_ref = PCH.calculateCurrentMagnitudeAndPhase()
    currentReference = CurrentReference(sampling_rate, cont_horizon, I_ref_peak, phi_ref, f_ref)
    mpc = MPCSSolver(cont_horizon=cont_horizon, lambda_switch=0.1, use_gurobi=use_gurobi)
    s0 = [True]*cont_horizon if phase_num==1 else [(1,1,1)]*cont_horizon

    if phase_num == 1:
        s_traj, i_a_traj, i_ref_traj, t_sim, cost_func_val = sim_executor(load,
                                                                          inverter,
                                                                          mpc,
                                                                          currentReference,
                                                                          s0,
                                                                          t_0,
                                                                          i_a_0,
                                                                          sampling_rate,
                                                                          sim_time,
                                                                          phase_num,
                                                                          i_b_0,
                                                                          i_c_0)
        # === Plot 1: switching & current ===
        fig1 = make_subplots(
            rows=2, cols=1, subplot_titles=("Switching Signal","Load Current"), vertical_spacing=0.12
        )
        fig1.add_trace(go.Scatter(x=t_sim, y=s_traj, mode="lines", name="Switching Signal s", line=dict(shape="hv")), row=1, col=1)
        fig1.update_yaxes(title_text="s", row=1, col=1, range=[-0.05,1.05])
        fig1.add_trace(go.Scatter(x=t_sim, y=i_a_traj, mode="lines", name="Load Current i_a"), row=2, col=1)
        fig1.add_trace(go.Scatter(x=t_sim, y=i_ref_traj, mode="lines", line=dict(dash="dash"), name="Reference Current i_a_ref"), row=2, col=1)
        fig1.update_yaxes(title_text="i (A)", row=2, col=1)
        fig1.update_xaxes(title_text="Time (s)", row=2, col=1)
        fig1.update_layout(height=600, width=600, template="simple_white", showlegend=True,
                           margin=dict(l=60,r=30,t=60,b=60),
                           legend=dict(x=0.02,y=0.98,bgcolor="rgba(255,255,255,0.5)"))
        fig1.show()
        # Plot 2: Cost
        fig2 = go.Figure()
        fig2.add_trace(go.Scatter(x=t_sim, y=cost_func_val, mode="lines", name="Cost Function Value"))
        fig2.update_layout(xaxis_title="Time (s)", yaxis_title="Cost Function Value", height=400, width=600,
                           template="simple_white", margin=dict(l=60,r=30,t=60,b=60),
                           legend=dict(x=0.02,y=0.98,bgcolor="rgba(255,255,255,0.5)"))
        fig2.show()
        return fig1, fig2
    else:
        # three-phase
        s_traj, (i_a_traj,i_b_traj,i_c_traj), (i_ref_a_traj,i_ref_b_traj,i_ref_c_traj), t_sim, cost_func_val = sim_executor(
            load, inverter, mpc, currentReference, s0, t_0, i_a_0, sampling_rate, sim_time, phase_num, i_b_0, i_c_0
        )
        ia,ib,ic = (i_a_traj,i_b_traj,i_c_traj)
        ira,irb,irc = (i_ref_a_traj,i_ref_b_traj,i_ref_c_traj)
        ialpha_ref, ibeta_ref = clarke_power_invariant(ira, irb, irc)
        ialpha, ibeta = clarke_power_invariant(ia, ib, ic)
        # Plot: switching signals and phase currents
        fig1 = make_subplots(rows=2, cols=1, subplot_titles=("Switching Signals","Phase Currents (meas & ref)"), vertical_spacing=0.12)
        # unpack s_traj list of tuples to series
        sa=[s[0] for s in s_traj]; sb=[s[1] for s in s_traj]; sc=[s[2] for s in s_traj]
        fig1.add_trace(go.Scatter(x=t_sim, y=sa, mode="lines", name="s_a", line=dict(shape="hv")), row=1, col=1)
        fig1.add_trace(go.Scatter(x=t_sim, y=sb, mode="lines", name="s_b", line=dict(shape="hv")), row=1, col=1)
        fig1.add_trace(go.Scatter(x=t_sim, y=sc, mode="lines", name="s_c", line=dict(shape="hv")), row=1, col=1)
        fig1.update_yaxes(title_text="s", row=1, col=1, range=[-0.05,1.05])
        fig1.add_trace(go.Scatter(x=t_sim, y=ia, mode="lines", name="i_a", line=dict(color="red")), row=2, col=1)
        fig1.add_trace(go.Scatter(x=t_sim, y=ib, mode="lines", name="i_b", line=dict(color="blue")), row=2, col=1)
        fig1.add_trace(go.Scatter(x=t_sim, y=ic, mode="lines", name="i_c", line=dict(color="green")), row=2, col=1)
        fig1.add_trace(go.Scatter(x=t_sim, y=ira, mode="lines", line=dict(dash="dash", color="red"), name="i_a_ref"), row=2, col=1)
        fig1.add_trace(go.Scatter(x=t_sim, y=irb, mode="lines", line=dict(dash="dash", color="blue"), name="i_b_ref"), row=2, col=1)
        fig1.add_trace(go.Scatter(x=t_sim, y=irc, mode="lines", line=dict(dash="dash", color="green"), name="i_c_ref"), row=2, col=1)
        fig1.update_yaxes(title_text="i (A)", row=2, col=1)
        fig1.update_xaxes(title_text="Time (s)", row=2, col=1)
        fig1.update_layout(height=650, width=850, template="simple_white", showlegend=True,
                           margin=dict(l=60,r=30,t=60,b=60),
                           legend=dict(x=0.02,y=0.98,bgcolor="rgba(255,255,255,0.5)"))
        fig1.show()
        # Plot 2: Cost
        fig2 = go.Figure()
        fig2.add_trace(go.Scatter(x=t_sim, y=cost_func_val, mode="lines", name="Cost Function Value"))
        fig2.update_layout(xaxis_title="Time (s)", yaxis_title="Cost Function Value", height=400, width=600,
                           template="simple_white", margin=dict(l=60,r=30,t=60,b=60),
                           legend=dict(x=0.02,y=0.98,bgcolor="rgba(255,255,255,0.5)"))
        fig2.show()
        # Plot 3: Clarke plane currents
        fig3 = go.Figure()
        fig3.add_trace(go.Scatter(x=ialpha_ref, y=ibeta_ref, mode="lines", name="Ref Current", line=dict(dash="dash")))
        fig3.add_trace(go.Scatter(x=ialpha, y=ibeta, mode="lines", name="Measured Current"))
        fig3.update_layout(xaxis_title="i_alpha (A)", yaxis_title="i_beta (A)", height=500, width=500,
                           template="simple_white", margin=dict(l=60,r=30,t=60,b=60))
        fig3.show()
        # Plot 4: alpha/beta vs time
        fig4 = go.Figure()
        fig4.add_trace(go.Scatter(x=t_sim, y=ialpha, mode="lines", name="i_alpha"))
        fig4.add_trace(go.Scatter(x=t_sim, y=ibeta, mode="lines", name="i_beta"))
        fig4.add_trace(go.Scatter(x=t_sim, y=ialpha_ref, mode="lines", name="i_alpha_ref", line=dict(dash="dash")))
        fig4.add_trace(go.Scatter(x=t_sim, y=ibeta_ref, mode="lines", name="i_beta_ref", line=dict(dash="dash")))
        fig4.update_layout(xaxis_title="Time (s)", yaxis_title="Current (A)", height=400, width=600,
                           template="simple_white", margin=dict(l=60,r=30,t=60,b=60))
        fig4.show()
        return fig1, fig2

if __name__ == "__main__":
    simPowerConvControlSyst()