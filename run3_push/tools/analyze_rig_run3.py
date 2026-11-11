"""
tools/analyze_rig_run3.py  —  third real-rig run (setpoint 36 °C, cold start, one supply disturbance).

    python tools/analyze_rig_run3.py [--log results/final_rig_run3/run_rl_36C_coldstart_disturbance.csv]

Phases (found from the log):
    phase A  0 -> t1 : cold start from room temperature to 36 °C on a weak supply (~3.5 W available), then hold
    dist. 1  t1 -> end: supply raised -> available heater power ~3.6 W -> ~6 W
Heater power available each second: P_full = R·(I/duty)², R measured at 100 % duty (the logged V·I overstates power
at partial duty because the supply sags only while the heater is on).

Standard methods (gains commissioned once at 4 V from the step-test model, as in runs 1 and 2) are run on a digital
twin calibrated to THIS run (K0, τ, θ fitted on the whole run), driven by the same P_full(t), from the same cold start.
The agent is run on the same twin too (like-for-like), starting from the gains it really started with.

Outputs in results/final_rig_run3/: stats.md, stats.json, overview.png
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)
import thermal_pid_env as E                                   # noqa: E402
from plant import PID, Plant, PlantConfig                     # noqa: E402
from tuning_rules import all_rules                            # noqa: E402

OUT = os.path.join("results", "final_rig_run3")
ap = argparse.ArgumentParser()
ap.add_argument("--log", default=os.path.join(OUT, "run_rl_36C_coldstart_disturbance.csv"))
ap.add_argument("--model", default=os.path.join("models", "td3_best", "best_model.zip"))
a = ap.parse_args()
os.makedirs(OUT, exist_ok=True)

d = pd.read_csv(a.log)
SP = float(d.SP_C.iloc[-1])
T_END = int(d.t_s.iloc[-1])
grid = np.arange(T_END + 1)
NOM = PlantConfig.nominal()
Tamb = float(d.T_C.iloc[:5].median())

# ---------------------------------------------------------------- 1. available heater power and the disturbance
ok = d.PWM_pct > 15
R_HEATER = float((d.V[d.PWM_pct > 99] / d.I_A[d.PWM_pct > 99]).median())
duty = (d.PWM_pct / 100).clip(lower=1e-3)
P_heater = np.where(d.PWM_pct > 0.5, R_HEATER * d.I_A ** 2 / duty, 0.0)
pf = (R_HEATER * (d.I_A / duty) ** 2).where(ok)
Pfull = pd.Series(np.interp(grid, d.t_s[ok], pf[ok]), index=grid).rolling(31, center=True, min_periods=1).median().values
base_P = float(np.median(Pfull[1200:3600]))
t1 = int(np.argmax((grid > 1800) & (Pfull > base_P * 1.3)))     # first second the available power jumps
Vt = np.sqrt(np.clip(Pfull, 0.05, None) * NOM.R)
T_REACH0 = 900                                                # phase A "hold" part scored from 15 min

# ---------------------------------------------------------------- 2. calibrate the twin to this run
Pg = np.interp(grid, d.t_s, P_heater)
Tg = np.interp(grid, d.t_s, d.T_C)
beta = NOM.beta


def sim_power(K0, tau, theta):
    x = Tg[0] - Tamb
    buf, out = [0.0] * int(round(theta)), np.empty(len(Pg))
    for i, p in enumerate(Pg):
        buf.append(p)
        x += (K0 * buf.pop(0) - x * (1 + beta * x)) / tau
        out[i] = Tamb + x
    return out


fit = least_squares(lambda z: sim_power(*z) - Tg, [NOM.K0, NOM.tau, NOM.theta], bounds=([1, 100, 5], [30, 5000, 200]))
K0c, tauc, thc = fit.x
rmse_cal = float(np.sqrt(np.mean(fit.fun ** 2)))
TRUE = PlantConfig(K0=K0c, beta=beta, tau=tauc, theta=thc, R=NOM.R, T_amb=Tamb, noise_std=0.02, quant=0.01)

# ---------------------------------------------------------------- 3. controllers on the calibrated twin
from stable_baselines3 import TD3                             # noqa: E402
agent = TD3.load(a.model, device="cpu")
OBS = PlantConfig.nominal(T_amb=Tamb)                         # live_tuner.py uses the measured ambient


def run(g0, use_agent=False, seed=1):
    p = Plant(TRUE, Vt[0], T0=Tamb, rng=np.random.default_rng(seed))
    pid = PID(g0["Kp"], g0["Ki"], g0["Kd"])
    meas = p.measure(); win = []; Tp = meas; T = []; U = []
    for t in range(T_END + 1):
        p.V = Vt[t]
        u = pid(SP, meas); p.step(u); meas = p.measure()
        win.append(u); T.append(p.T); U.append(u)
        if use_agent and len(win) == E.RL_EVERY:
            Vo = float(np.clip(Vt[t], 1.8, 5.5)); base = E.base_gains(OBS, Vo, SP)
            z = E.z_from_gains((pid.Kp, pid.Ki, pid.Kd), base)
            head = (SP - OBS.T_amb) / (OBS.T_max(Vo) - OBS.T_amb)
            obs = np.array([(SP - meas) / 10, np.clip(meas - Tp, -10, 10), np.mean(win) / 100 - .5,
                            np.mean(np.abs(np.diff(win))) / 50, *z, (Vo - 3.5) / 2, np.clip(head, 0, 2),
                            (meas - OBS.T_amb) / 50], np.float32)
            act = agent.predict(obs, deterministic=True)[0]
            z = np.clip(z + E.STEP_Z * np.clip(act, -1, 1), -1, 1)
            pid.set_tunings(*E.gains_from_z(z, base)); win = []; Tp = meas
    return np.array(T), np.array(U)


def settle(e, band):
    o = np.where(np.abs(e) > band)[0]
    return 0.0 if not len(o) else (float((o[-1] + 1) / 60) if o[-1] < len(e) - 1 else None)


def metrics(T, U):
    e = SP - T
    hit = np.where(T[:t1] >= SP - 0.1)[0]
    eA, eH, eD = e[:t1], e[T_REACH0:t1], e[t1:]
    return dict(
        IAE_total=float(np.abs(e).sum()), IAE_A=float(np.abs(eA).sum()), IAE_D1=float(np.abs(eD).sum()),
        reach_min=float(hit[0] / 60) if len(hit) else None,
        overshoot_A=float(max(0.0, -eA.min())), settle_A_025=settle(eA, 0.25),
        hold_rms=float(np.sqrt(np.mean(eH ** 2))), hold_band=float(np.max(np.abs(eH))),
        D1_peak=float(np.max(np.abs(eD))), D1_settle_025=settle(eD, 0.25), D1_settle_01=settle(eD, 0.1),
        rms_last5=float(np.sqrt(np.mean(e[-300:] ** 2))),
        du=None if U is None else float(np.mean(np.abs(np.diff(U)))))


Treal = Tg
Ureal = np.interp(grid, d.t_s, d.PWM_pct)
res = {"TD3 agent — REAL RIG": metrics(Treal, None)}
res["TD3 agent — REAL RIG"]["du"] = float(np.mean(np.abs(np.diff(d.PWM_pct.values))) / np.mean(np.diff(d.t_s.values)))
traj = {}
g0 = d.iloc[0]
T_, U_ = run({"Kp": float(g0.Kp), "Ki": float(g0.Ki), "Kd": float(g0.Kd)}, use_agent=True)
res["TD3 agent — twin"] = metrics(T_, U_); traj["TD3 agent — twin"] = T_
rules = all_rules(*NOM.fopdt_at(4.0, SP))
rules["MATLAB pidtune (PI)"] = {"Kp": 4.69969, "Ki": 0.0090637, "Kd": 0.0}
for k, g in rules.items():
    T_, U_ = run(g); res[f"{k} — twin"] = metrics(T_, U_); traj[f"{k} — twin"] = T_

run_info = dict(setpoint=SP, ambient=Tamb, t1_s=t1, R_heater=R_HEATER,
                P_before_W=float(np.median(Pfull[1200:t1 - 10])), P_after_W=float(np.median(Pfull[t1 + 300:])),
                calibrated=dict(K0=K0c, tau=tauc, theta=thc, rmse=rmse_cal), gain_updates=int(d.sent.notna().sum()),
                gains_hold=dict(Kp=float(d.Kp[(d.t_s > 900) & (d.t_s < t1)].median()), Ki=float(d.Ki[(d.t_s > 900) & (d.t_s < t1)].median()),
                                Kd=float(d.Kd[(d.t_s > 900) & (d.t_s < t1)].median())),
                gains_end=dict(Kp=float(d.Kp.iloc[-1]), Ki=float(d.Ki.iloc[-1]), Kd=float(d.Kd.iloc[-1])))
json.dump(dict(metrics=res, run=run_info), open(os.path.join(OUT, "stats.json"), "w"), indent=1)

f = lambda v, n=1: "never" if v is None else f"{v:,.{n}f}"   # noqa: E731
L = [f"# Rig run 3 — setpoint {SP:.0f} °C, cold start + one supply disturbance (t1 = {t1} s)\n",
     f"Heater power available: {run_info['P_before_W']:.2f} W -> {run_info['P_after_W']:.2f} W at t1. Ambient {Tamb:.2f} °C. "
     f"Twin calibrated on the whole run: K0 {K0c:.2f} °C/W, τ {tauc:.0f} s, θ {thc:.0f} s, RMSE {rmse_cal:.2f} °C.\n",
     "| Controller | IAE total | reach SP (min) | overshoot (°C) | settle ±0.25 (min) | hold RMS 15 min→t1 (°C) | D1 peak (°C) | D1 back ±0.25 (min) | D1 back ±0.1 (min) | RMS last 5 min (°C) | mean abs du (%/s) |",
     "|---|---|---|---|---|---|---|---|---|---|---|"]
for k, v in res.items():
    L.append(f"| {k} | {v['IAE_total']:,.0f} | {f(v['reach_min'])} | {v['overshoot_A']:.2f} | {f(v['settle_A_025'])} | {v['hold_rms']:.3f} | "
             f"{v['D1_peak']:.2f} | {f(v['D1_settle_025'])} | {f(v['D1_settle_01'])} | {v['rms_last5']:.3f} | {f(v['du'])} |")
open(os.path.join(OUT, "stats.md"), "w").write("\n".join(L) + "\n")
print("\n".join(L))

# ---------------------------------------------------------------- 4. figure
tm = grid / 60
fig, ax = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True, gridspec_kw=dict(height_ratios=[3, 1.2, 1.2]))
for k, col in (("SIMC — twin", "#9aa0ad"), ("Cohen-Coon — twin", "#b3a6d6"), ("MATLAB pidtune (PI) — twin", "#d9a3a3")):
    ax[0].plot(tm, traj[k], color=col, lw=1.2, label=k.replace(" — twin", " (fixed gains, calibrated twin)"))
ax[0].plot(tm, Treal, color="#2e86de", lw=2.2, label="TD3 agent — REAL RIG")
ax[0].axhline(SP, color="k", ls="--", lw=0.9, label=f"setpoint {SP:.0f} °C")
ax[0].axhspan(SP - 0.25, SP + 0.25, color="#2e86de", alpha=0.07)
for x in ax:
    x.axvline(t1 / 60, color="#e4572e", lw=1.2)
ax[0].text(t1 / 60 + 0.4, SP - 4.5, f"supply up: {run_info['P_before_W']:.1f} → {run_info['P_after_W']:.1f} W", color="#e4572e",
           fontsize=10, fontweight="bold")
ax[0].set_ylim(Tamb - 0.5, SP + 1.5); ax[0].set_ylabel("T [°C]"); ax[0].legend(loc="lower right", fontsize=8.5)
ax[0].set_title(f"Real rig run 3: cold start to {SP:.0f} °C on a weak supply, then supply raised")
ax[1].plot(d.t_s / 60, d.PWM_pct, color="#2e86de", lw=0.8); ax[1].set_ylabel("PWM [%]")
for g, col in (("Kp", "#2e86de"), ("Ki", "#1f9d63"), ("Kd", "#e4572e")):
    ax[2].semilogy(d.t_s / 60, d[g], color=col, lw=1.2, label=g)
ax[2].set_ylabel("gains (log)"); ax[2].set_xlabel("minutes"); ax[2].legend(loc="upper right", fontsize=8.5, ncol=3)
for x in ax:
    x.grid(alpha=.3)
plt.tight_layout(); plt.savefig(os.path.join(OUT, "overview.png"), dpi=120); plt.close()
print("wrote", OUT)
