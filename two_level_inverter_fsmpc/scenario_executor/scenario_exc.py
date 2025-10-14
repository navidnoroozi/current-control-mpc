# === scenario_executor/scenario_exc.py ===
import math

def sim_executor(
    load,
    inverter,
    mpc,
    currentReference,
    s0,
    t_0=0.0,
    i_a_0=0.0,
    sampling_rate=1e-4,
    sim_time=0.1,
    phase_num=1,
    i_b0=0.0,
    i_c0=0.0,
):
    """
    Phase-aware simulation loop.

    Returns
    -------
    phase_num == 1:
        (s_traj, i_a_traj, i_ref_a_traj, t_sim, cost_func_val)
    phase_num == 3:
        (s_traj, (i_a_traj, i_b_traj, i_c_traj),
         (i_ref_a_traj, i_ref_b_traj, i_ref_c_traj), t_sim, cost_func_val)
    """
    steps = max(0, int(round((sim_time - t_0) / sampling_rate)))
    current_time = t_0

    # init currents
    i_a = float(i_a_0)
    i_b = float(i_b0)
    i_c = float(i_c0)

    # logs
    s_traj = []
    cost_func_val = []
    t_sim = []

    i_a_traj, i_ref_a_traj = [], []
    i_b_traj, i_ref_b_traj = [], []
    i_c_traj, i_ref_c_traj = [], []

    for _ in range(steps):

        # === MPC decision on full horizon, but apply only the first action ===
        if phase_num == 1:
            s_seq, J = mpc.solveMPC(
                inverter, load, currentReference, current_time,
                i_init=i_a, s0=s0, phase_num=1
            )
            sa = int(s_seq[0])

            # Inverter pole voltage for phase a (single leg)
            va = inverter.V_dc * (2*sa - 1) / 2.0

            # back-emf (consistent sine convention)
            w = 2.0 * math.pi * load.f_backemf
            ea = load.V_backemf * math.sin(w*current_time)

            # integrate RL one step
            i_a += ((va - ea) - load.R * i_a) / load.L * sampling_rate

            # log action and signals
            s_traj.append(sa)
            i_a_traj.append(i_a)

            # scalar ref at current sample
            iref_now = currentReference.generateRefTrajectory(current_time)[0]
            i_ref_a_traj.append(float(iref_now))

        else:  # === phase_num == 3 ===
            s_seq, J = mpc.solveMPC(
                inverter, load, currentReference, current_time,
                i_init=(i_a, i_b, i_c), s0=s0, phase_num=3
            )
            sa, sb, sc = map(int, s_seq[0])

            # Inverter pole voltages (per leg)
            vaN = inverter.V_dc * (2*sa - 1) / 2.0
            vbN = inverter.V_dc * (2*sb - 1) / 2.0
            vcN = inverter.V_dc * (2*sc - 1) / 2.0

            # === ZERO-SEQUENCE REMOVAL (three-wire) ===
            v0  = (vaN + vbN + vcN) / 3.0
            va, vb, vc = vaN - v0, vbN - v0, vcN - v0

            # Balanced back-emf set (sum = 0)
            w = 2.0 * math.pi * load.f_backemf
            ea = load.V_backemf * math.sin(w*current_time)
            eb = load.V_backemf * math.sin(w*current_time - 2.0*math.pi/3.0)
            ec = load.V_backemf * math.sin(w*current_time + 2.0*math.pi/3.0)

            # integrate three RL branches
            i_a += ((va - ea) - load.R*i_a) / load.L * sampling_rate
            i_b += ((vb - eb) - load.R*i_b) / load.L * sampling_rate
            i_c += ((vc - ec) - load.R*i_c) / load.L * sampling_rate

            s_traj.append((sa, sb, sc))
            i_a_traj.append(i_a); i_b_traj.append(i_b); i_c_traj.append(i_c)

            # scalar refs at this sample
            ia_ref_seq, ib_ref_seq, ic_ref_seq = currentReference.generateThreePhaseRefs(current_time)
            i_ref_a_traj.append(float(ia_ref_seq[0]))
            i_ref_b_traj.append(float(ib_ref_seq[0]))
            i_ref_c_traj.append(float(ic_ref_seq[0]))

        # log cost & time
        cost_func_val.append(float(J))
        t_sim.append(current_time)

        # warm-start for next call: repeat first action across horizon
        if phase_num == 1:
            s0 = [bool(s_traj[-1])] * mpc.cont_horizon
        else:
            psa, psb, psc = s_traj[-1]
            s0 = [(bool(psa), bool(psb), bool(psc))] * mpc.cont_horizon

        current_time += sampling_rate

    if phase_num == 1:
        return s_traj, i_a_traj, i_ref_a_traj, t_sim, cost_func_val
    else:
        return (
            s_traj,
            (i_a_traj, i_b_traj, i_c_traj),
            (i_ref_a_traj, i_ref_b_traj, i_ref_c_traj),
            t_sim,
            cost_func_val,
        )
