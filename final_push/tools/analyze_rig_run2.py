"""
tools/analyze_rig_run2.py  —  second real-rig run (setpoint 37 °C, after the ambient fix in live_tuner.py).

    python tools/analyze_rig_run2.py [--log results/final_rig_run2/run_rl_37C_voltage_disturbances.csv]

The first ~59 min are ignored for scoring: at the starting supply (~2.1 V under load, 1.85 W) the heater cannot
reach 37 °C, so every controller simply sits at 100 % PWM. What is scored are the two supply changes:
    disturbance 1 (t1): supply raised  -> available heater power 1.85 W -> ~8.5 W
    disturbance 2 (t2): supply lowered -> ~8.5 W -> ~4.5 W
t1, t2 are found from the logged supply voltage. The supply sags under load, so the disturbance is described by the
heater power actually available each second (P_full = measured power / duty), interpolated where the duty is low.

Standard methods are run on a digital twin calibrated to THIS run (K0, τ, θ fitted to measured power -> temperature
from 10 min before t1 to the end), driven with the same P_full(t). Every controller starts at t1 from the rig's real
state at that moment (temperature, heater saturated at 100 %, integral term at its 100 % limit), and is scored on the
same windows. The agent is run on the same twin too (starting from the gains it really had at t1).

Outputs in results/final_rig_run2/: stats.md, stats.json, overview.png, comparison.png
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

OUT = os.path.join("results", "final_rig_run2")
ap = argparse.ArgumentParser()
ap.add_argument("--log", default=os.path.join(OUT, "run_rl_37C_voltage_disturbances.csv"))
ap.add_argument("--model", default=os.path.join("models", "td3_best", "best_model.zip"))
a = ap.parse_args()
os.makedirs(OUT, exist_ok=True)

d = pd.read_csv(a.log)
SP = float(d.SP_C.iloc[-1])
T_END = int(d.t_s.iloc[-1])
grid = np.arange(T_END + 1)
NOM = PlantConfig.nominal()
Tamb = float(d.T_C.iloc[:5].median())

# ---------------------------------------------------------------- 1. disturbances and available heater power
t1 = float(d.t_s[(d.t_s > 600) & (d.V > d.V.iloc[:600].median() + 0.8)].iloc[0])
after = d[d.t_s > t1 + 600]
t2 = float(after.t_s[after.V.rolling(20, min_periods=1).max() < after.V.iloc[:300].median() - 0.6].iloc[0])
t1, t2 = int(round(t1)), int(round(t2))
ok = d.PWM_pct > 15
# Heater power: the logged P = mean(V)·mean(I) overestimates it at partial duty (the supply sags only while the heater
# is on). Use the heater resistance measured at 100 % duty: P_full = R·(mean(I)/duty)², P = R·mean(I)²/duty.
R_HEATER = float((d.V[d.PWM_pct > 99] / d.I_A[d.PWM_pct > 99]).median())
duty = (d.PWM_pct / 100).clip(lower=1e-3)
P_heater = np.where(d.PWM_pct > 0.5, R_HEATER * d.I_A ** 2 / duty, 0.0)
pf = (R_HEATER * (d.I_A / duty) ** 2).where(ok)
s = pd.Series(np.interp(grid, d.t_s[ok], pf[ok]), index=grid).rolling(31, center=True, min_periods=1).median()
Pfull = s.values                                              # W available at 100 % duty, each second
Vt = np.sqrt(np.clip(Pfull, 0.05, None) * NOM.R)              # model supply giving that power

# ---------------------------------------------------------------- 2. calibrate the twin to this run
Pg = np.interp(grid, d.t_s, P_heater)
Tg = np.interp(grid, d.t_s, d.T_C)
beta = NOM.beta
c0 = t1 - 600                                                 # calibration window: 10 min before t1 -> end


def sim_power(K0, tau, theta):
    x = Tg[c0] - Tamb
    buf, out = [Pg[c0]] * int(round(theta)), np.empty(len(Pg) - c0)
    for i, p in enumerate(Pg[c0:]):
        buf.append(p)
        x += (K0 * buf.pop(0) - x * (1 + beta * x)) / tau
        out[i] = Tamb + x
    return out


fit = least_squares(lambda z: sim_power(*z) - Tg[c0:], [NOM.K0, NOM.tau, NOM.theta], bounds=([1, 100, 5], [30, 5000, 200]))
K0c, tauc, thc = fit.x
rmse_cal = float(np.sqrt(np.mean(fit.fun ** 2)))
TRUE = PlantConfig(K0=K0c, beta=beta, tau=tauc, theta=thc, R=NOM.R, T_amb=Tamb, noise_std=0.02, quant=0.01)

# ---------------------------------------------------------------- 3. controllers on the calibrated twin
from stable_baselines3 import TD3                             # noqa: E402
agent = TD3.load(a.model, device="cpu")
OBS = PlantConfig.nominal(T_amb=Tamb)                         # as live_tuner.py now does (measured ambient)


def run(g0, use_agent=False, seed=1):
    """Start at t1 from the rig's real state: T(t1), heater at 100 % for the last θ s, integral at its 100 % limit."""
    p = Plant(TRUE, Vt[t1], T0=float(Tg[t1]), rng=np.random.default_rng(seed))
    p.buf = [float(Pfull[t1 - 5])] * len(p.buf)
    pid = PID(g0["Kp"], g0["Ki"], g0["Kd"]); pid.iterm = 100.0; pid.last_y = float(Tg[t1])
    meas = p.measure(); win = []; Tp = meas; T = list(Tg[:t1]); U = [100.0] * t1
    for t in range(t1, T_END + 1):
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


def window_stats(T, a0, b0):
    e = SP - T[a0:b0]
    r = dict(IAE=float(np.abs(e).sum()), peak_dev=float(np.max(np.abs(e))), overshoot=float(max(0.0, -e.min())),
             undershoot=float(max(0.0, e.max())), rms_last5=float(np.sqrt(np.mean(e[-300:] ** 2))))
    for band in (0.5, 0.25):
        o = np.where(np.abs(e) > band)[0]
        r[f"settle_{band}"] = 0.0 if not len(o) else (float((o[-1] + 1) / 60) if o[-1] < len(e) - 1 else None)
    return r


def metrics(T):
    return {"D1": window_stats(T, t1, t2), "D2": window_stats(T, t2, len(T)), "scored": window_stats(T, t1, len(T))}


Treal = Tg
res = {"TD3 agent — REAL RIG": metrics(Treal)}
traj = {}
g_t1 = d.iloc[(d.t_s - t1).abs().idxmin()]
T_, _ = run({"Kp": float(g_t1.Kp), "Ki": float(g_t1.Ki), "Kd": float(g_t1.Kd)}, use_agent=True)
res["TD3 agent — twin"] = metrics(T_); traj["TD3 agent — twin"] = T_
rules = all_rules(*NOM.fopdt_at(4.0, SP))                     # commissioned once from the step-test model
rules["MATLAB pidtune (PI)"] = {"Kp": 4.69969, "Ki": 0.0090637, "Kd": 0.0}
for k, g in rules.items():
    T_, _ = run(g); res[f"{k} — twin"] = metrics(T_); traj[f"{k} — twin"] = T_

run_info = dict(setpoint=SP, ambient=Tamb, t1_s=t1, t2_s=t2,
                P_before_W=float(np.median(Pfull[t1 - 300:t1 - 10])), P_after1_W=float(np.median(Pfull[t1 + 300:t2 - 10])),
                P_after2_W=float(np.median(Pfull[t2 + 300:])),
                calibrated=dict(K0=K0c, tau=tauc, theta=thc, rmse=rmse_cal),
                gain_updates=int(d.sent.notna().sum()))
json.dump(dict(metrics=res, run=run_info), open(os.path.join(OUT, "stats.json"), "w"), indent=1)

f = lambda v, n=1: "never" if v is None else f"{v:,.{n}f}"   # noqa: E731
L = [f"# Rig run 2 — setpoint {SP:.0f} °C, two supply disturbances (scored from t1 = {t1} s)\n",
     f"Heater power available: {run_info['P_before_W']:.2f} W -> {run_info['P_after1_W']:.1f} W at t1, -> {run_info['P_after2_W']:.1f} W at t2 = {t2} s. "
     f"Twin calibrated to this run (from t1 - 10 min): K0 {K0c:.2f} °C/W, τ {tauc:.0f} s, θ {thc:.0f} s, RMSE {rmse_cal:.2f} °C.\n",
     "| Controller | IAE scored | D1 peak (°C) | D1 overshoot (°C) | D1 settle ±0.25 (min) | D2 peak (°C) | D2 settle ±0.25 (min) | RMS last 5 min (°C) |",
     "|---|---|---|---|---|---|---|---|"]
for k, v in res.items():
    L.append(f"| {k} | {v['scored']['IAE']:,.0f} | {v['D1']['peak_dev']:.2f} | {v['D1']['overshoot']:.2f} | {f(v['D1']['settle_0.25'])} | "
             f"{v['D2']['peak_dev']:.2f} | {f(v['D2']['settle_0.25'])} | {v['D2']['rms_last5']:.3f} |")
open(os.path.join(OUT, "stats.md"), "w").write("\n".join(L) + "\n")
print("\n".join(L))

# ---------------------------------------------------------------- 4. figures
w0 = t1 - 120
tm = (grid - t1) / 60
fig, ax = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True, gridspec_kw=dict(height_ratios=[3, 1.2, 1.2]))
for k, col in (("SIMC — twin", "#9aa0ad"), ("Cohen-Coon — twin", "#b3a6d6"), ("MATLAB pidtune (PI) — twin", "#d9a3a3")):
    ax[0].plot(tm[w0:], traj[k][w0:], color=col, lw=1.2, label=k.replace(" — twin", " (fixed gains, calibrated twin)"))
ax[0].plot(tm[w0:], Treal[w0:], color="#2e86de", lw=2.2, label="TD3 agent — REAL RIG")
ax[0].axhline(SP, color="k", ls="--", lw=0.9, label=f"setpoint {SP:.0f} °C")
ax[0].axhspan(SP - 0.25, SP + 0.25, color="#2e86de", alpha=0.07)
for tt, txt in ((t1, f"supply up: {run_info['P_before_W']:.1f} → {run_info['P_after1_W']:.1f} W"),
                (t2, f"supply down: → {run_info['P_after2_W']:.1f} W")):
    for x in ax:
        x.axvline((tt - t1) / 60, color="#e4572e", lw=1.2)
    ax[0].text((tt - t1) / 60 + 0.4, SP + 2.4, txt, color="#e4572e", fontsize=10, fontweight="bold")
ax[0].set_ylim(SP - 2.0, SP + 3.0); ax[0].set_ylabel("T [°C]"); ax[0].legend(loc="lower right", fontsize=8.5)
ax[0].set_title(f"Real rig run 2: {SP:.0f} °C setpoint, supply raised then lowered")
m = d.t_s >= w0
ax[1].plot((d.t_s[m] - t1) / 60, d.PWM_pct[m], color="#2e86de", lw=0.8); ax[1].set_ylabel("PWM [%]")
for g, col in (("Kp", "#2e86de"), ("Ki", "#1f9d63"), ("Kd", "#e4572e")):
    ax[2].semilogy((d.t_s[m] - t1) / 60, d[g][m], color=col, lw=1.2, label=g)
ax[2].set_ylabel("gains (log)"); ax[2].set_xlabel("minutes after disturbance 1"); ax[2].legend(loc="upper right", fontsize=8.5, ncol=3)
for x in ax:
    x.grid(alpha=.3)
plt.tight_layout(); plt.savefig(os.path.join(OUT, "overview.png"), dpi=120); plt.close()
print("wrote", OUT)
