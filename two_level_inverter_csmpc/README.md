# Continuous-Set MPC with PWM (Fixed)

This package implements a continuous-set MPC for a two-level converter using a **triangle-carrier PWM**.
Key changes vs. your original structure:

1. **Averaged-model MPC prediction**: v ≈ Vdc * u inside the cost/prediction (no switching in the horizon).
2. **PWM only on plant update**: when `with_pwm=True`, we synthesize a switching waveform **with sub-steps**
   inside each control step and integrate the load across those sub-steps.
3. **Bipolar PWM output**: ±Vdc comparator, using duty d = (u+1)/2 clipped to [0,1].
4. **Aliasing avoided**: at least 20 samples per carrier period and 200 per control step.

## Layout

- pwm/pwm_gen.py — triangle PWM with sub-stepping (±Vdc) + averaged helper
- load/load_dyn_cal.py — RL load; now includes `calculateLoadDynamicsSubsteps(...)`
- cost_fun/cost_func_calc.py — averaged-model prediction in MPC cost
- mpc_contr/mpc_contr_calc.py — simple SciPy-based solver (bounds in [-1,1])
- scenario_executor/scenario_exc.py — loop that applies PWM for real plant update, averaged otherwise
- current_reference/current_ref_gen.py — sine reference
- inverter.py — averaged model helper (Vdc*u)
- main_fixed.py — quick demo runner

## Quick Start

```bash
pip install -r requirements.txt
python main_fixed.py
```

You should see u(t), the per-step averaged voltage from PWM, and current i_a tracking the reference.

## Notes
- Tune `carrier_freq` and `sampling_rate` such that `1/(fc*Ts) >= 20` (this code enforces a minimum).
- For fairness, the MPC cost **never** sees switching ripple; the plant does (when with_pwm=True).
- Extend to 3-phase by instantiating one PWM per leg and augmenting the load model.
