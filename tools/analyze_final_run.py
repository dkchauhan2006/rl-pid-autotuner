"""
tools/analyze_final_run.py  —  statistics of the final real-rig run of the v2 agent, and comparison with
the classical tuning rules under the SAME supply-voltage disturbances.

    python tools/analyze_final_run.py [--log results/final_rig_run/run_rl_35C_voltage_disturbances.csv]

Run (10 Nov 2026): setpoint 35 °C, cold start, agent starts from the firmware defaults (Kp 20, Ki 0.05, Kd 100).
Disturbances = supply voltage changed by hand during the run:
    phase A  t <  T1   supply ≈ 3.3 V        (cold start to 35 °C)
    phase B  T1 ≤ t < T2  supply ≈ 5.2 V     (disturbance 1: heater power ×3.5)
    phase C  t ≥ T2   supply ≈ 4.1 V         (disturbance 2: heater power ×0.6)
T1, T2 are found from the logged supply voltage.

The classical controllers cannot be run on the rig retroactively, so they are run on a digital twin that is
CALIBRATED TO THIS RUN (K0, τ, θ fitted to the measured heater power -> measured temperature) and driven with the
same heater power per phase. The agent is run on the same twin as well, so the like-for-like comparison is
twin-vs-twin; the real-rig numbers are shown next to it.

Outputs in results/final_rig_run/: stats.md, stats.json, overview.png, iae_by_phase.png
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

OUT = os.path.join("results", "final_rig_run")
ap = argparse.ArgumentParser()
ap.add_argument("--log", default=os.path.join(OUT, "run_rl_35C_voltage_disturbances.csv"))
ap.add_argument("--model", default=os.path.join("models", "td3_best", "best_model.zip"))
a = ap.parse_args()
os.makedirs(OUT, exist_ok=True)

d = pd.read_csv(a.log)
SP = float(d.SP_C.iloc[0])
T_END = int(d.t_s.iloc[-1])
grid = np.arange(T_END + 1)

# ---------------------------------------------------------------- 1. find the disturbances
Vroll = d.V.rolling(30, min_periods=1).max()
T1 = float(d.t_s[d.V > 4.5].iloc[0]) - 5                      # supply raised
after = d[d.t_s > T1 + 60]
T2 = float(after.t_s[after.V.rolling(30, min_periods=1).max() < 4.6].iloc[0]) - 30   # supply lowered
T1, T2 = int(round(T1)), int(round(T2))
phase = np.where(d.t_s < T1, "A", np.where(d.t_s < T2, "B", "C"))
Voc = {k: float(d.V[(phase == k) & (d.PWM_pct < 40)].quantile(0.9)) for k in "ABC"}     # ≈ open-circuit supply
Pfull = {k: float(np.median(d.P_W[(phase == k) & (d.PWM_pct > 10)] / (d.PWM_pct[(phase == k) & (d.PWM_pct > 10)] / 100)))
         for k in "ABC"}

# ---------------------------------------------------------------- 2. calibrate the twin to this run
Pg = np.interp(grid, d.t_s, d.P_W)
Tg = np.interp(grid, d.t_s, d.T_C)
NOM = PlantConfig.nominal()
beta, Tamb = NOM.beta, float(d.T_C.iloc[0])


def sim_power(K0, tau, theta):
    x, buf, out = 0.0, [0.0] * int(round(theta)), np.empty(len(Pg))
    for i, p in enumerate(Pg):
        buf.append(p)
        x += (K0 * buf.pop(0) - x * (1 + beta * x)) / tau
        out[i] = Tamb + x
    return out


fit = least_squares(lambda z: sim_power(*z) - Tg, [NOM.K0, NOM.tau, NOM.theta], bounds=([1, 100, 5], [30, 5000, 200]))
K0c, tauc, thc = fit.x
rmse_cal = float(np.sqrt(np.mean(fit.fun ** 2)))
TRUE = PlantConfig(K0=K0c, beta=beta, tau=tauc, theta=thc, R=NOM.R, T_amb=Tamb, noise_std=0.02, quant=0.01)
Vph = {k: float(np.sqrt(Pfull[k] * NOM.R)) for k in "ABC"}  # supply giving the same full heater power on the model


def V_at(t):
    return Vph["A"] if t < T1 else (Vph["B"] if t < T2 else Vph["C"])


# ---------------------------------------------------------------- 3. controllers on the calibrated twin
from stable_baselines3 import TD3                             # noqa: E402
agent = TD3.load(a.model, device="cpu")


def run(g0=None, use_agent=False, sched=None, seed=1):
    p = Plant(TRUE, V_at(0), T0=Tamb, rng=np.random.default_rng(seed))
    g = sched(V_at(0)) if sched else g0
    pid = PID(g["Kp"], g["Ki"], g["Kd"])
    meas = p.measure(); win = []; Tp = meas; T = []; U = []
    for t in range(T_END + 1):
        V = V_at(t); p.V = V
        if sched and t in (T1, T2):
            g = sched(V); pid.set_tunings(g["Kp"], g["Ki"], g["Kd"])
        u = pid(SP, meas); p.step(u); meas = p.measure()
        win.append(u); T.append(p.T); U.append(u)
        if use_agent and len(win) == E.RL_EVERY:                # exactly what live_tuner.py does
            Vo = float(np.clip(V, 1.8, 5.5)); base = E.base_gains(NOM, Vo, SP)
            z = E.z_from_gains((pid.Kp, pid.Ki, pid.Kd), base)
            head = (SP - NOM.T_amb) / (NOM.T_max(Vo) - NOM.T_amb)
            obs = np.array([(SP - meas) / 10, np.clip(meas - Tp, -10, 10), np.mean(win) / 100 - .5,
                            np.mean(np.abs(np.diff(win))) / 50, *z, (Vo - 3.5) / 2, np.clip(head, 0, 2),
                            (meas - NOM.T_amb) / 50], np.float32)
            act = agent.predict(obs, deterministic=True)[0]
            z = np.clip(z + E.STEP_Z * np.clip(act, -1, 1), -1, 1)
            pid.set_tunings(*E.gains_from_z(z, base)); win = []; Tp = meas
    return np.array(T), np.array(U)


def metrics(T, U):
    e = SP - T; out = {}
    for name, a0, b0 in (("A", 0, T1), ("B", T1, T2), ("C", T2, len(T)), ("all", 0, len(T))):
        es = e[a0:b0]
        r = dict(IAE=float(np.abs(es).sum()), ISE=float((es ** 2).sum()), ITAE=float((np.abs(es) * np.arange(len(es))).sum() / 1e3),
                 rms_last5=float(np.sqrt(np.mean(es[-300:] ** 2))), in025=float(np.mean(np.abs(es) <= 0.25) * 100),
                 in05=float(np.mean(np.abs(es) <= 0.5) * 100), mean_du=float(np.mean(np.abs(np.diff(U[a0:b0])))))
        if name in ("B", "C"):
            r["max_dev"] = float(np.max(np.abs(es)))
        for band in (0.5, 0.25):
            o = np.where(np.abs(es) > band)[0]
            r[f"recover_{band}"] = 0.0 if not len(o) else (float(o[-1] / 60) if o[-1] < len(es) - 1 else None)
        if name == "A":
            reached = np.where(T[a0:b0] >= SP - 0.05)[0]
            r["t_reach_min"] = float(reached[0] / 60) if len(reached) else None
            r["overshoot"] = float(max(0.0, T[a0:b0].max() - SP))
            i10 = np.argmax(T >= Tamb + 0.1 * (SP - Tamb)); i90 = np.argmax(T >= Tamb + 0.9 * (SP - Tamb))
            r["rise_10_90_min"] = float((i90 - i10) / 60) if T.max() >= Tamb + 0.9 * (SP - Tamb) else None
        out[name] = r
    return out


Treal = Tg; Ureal = np.interp(grid, d.t_s, d.PWM_pct)
res = {"TD3 agent — REAL RIG": metrics(Treal, Ureal)}
traj = {}
Ttw, Utw = run(g0={"Kp": 20.0, "Ki": 0.05, "Kd": 100.0}, use_agent=True)
res["TD3 agent — twin"] = metrics(Ttw, Utw); traj["TD3 agent — twin"] = Ttw
rules = all_rules(*NOM.fopdt_at(4.0, SP))                     # commissioned once from the step-test model
rules["MATLAB pidtune (PI)"] = {"Kp": 4.69969, "Ki": 0.0090637, "Kd": 0.0}
for k, g in rules.items():
    T_, U_ = run(g0=g); res[f"{k} — twin"] = metrics(T_, U_); traj[f"{k} — twin"] = T_
T_, U_ = run(sched=lambda V: all_rules(*TRUE.fopdt_at(V, SP))["SIMC"])
res["SIMC re-tuned on the true model at every voltage (oracle) — twin"] = metrics(T_, U_)

# ---------------------------------------------------------------- 4. real-run descriptives
gains = {k: dict(Kp=[float(d.Kp[phase == k].min()), float(d.Kp[phase == k].max()), float(d.Kp[phase == k].iloc[-1])],
                 Ki=[float(d.Ki[phase == k].min()), float(d.Ki[phase == k].max()), float(d.Ki[phase == k].iloc[-1])],
                 Kd=[float(d.Kd[phase == k].min()), float(d.Kd[phase == k].max()), float(d.Kd[phase == k].iloc[-1])])
         for k in "ABC"}
tail = d[(phase == "C")]
desc = dict(T1_s=T1, T2_s=T2, supply_open_circuit_V=Voc, heater_full_power_W=Pfull, twin_supply_V=Vph,
            calibrated=dict(K0=K0c, tau=tauc, theta=thc, rmse=rmse_cal, stepTest=dict(K0=NOM.K0, tau=NOM.tau, theta=NOM.theta)),
            gain_updates=int(d.sent.notna().sum()), gains=gains,
            phaseC_mean_err=float((SP - tail.T_C).mean()), phaseC_std=float(tail.T_C.std()),
            phaseC_band=[float(tail.T_C.min()), float(tail.T_C.max())],
            mean_pwm={k: float(d.PWM_pct[(phase == k)].tail(300).mean()) for k in "ABC"},
            logged_IAE_real=float(d.IAE_real.iloc[-1]), logged_IAE_twin_simc=float(d.IAE_twin_classical.iloc[-1]))
json.dump(dict(metrics=res, run=desc), open(os.path.join(OUT, "stats.json"), "w"), indent=1)

# ---------------------------------------------------------------- 5. markdown tables
f = lambda v, n=0: "never" if v is None else (f"{v:,.{n}f}")  # noqa: E731
lines = [f"# Final rig run — setpoint {SP:.0f} °C with two supply disturbances\n",
         f"Disturbance 1 at t = {T1} s: supply {Voc['A']:.2f} → {Voc['B']:.2f} V (heater full power {Pfull['A']:.1f} → {Pfull['B']:.1f} W). "
         f"Disturbance 2 at t = {T2} s: → {Voc['C']:.2f} V ({Pfull['C']:.1f} W). Run length {T_END / 60:.0f} min, {desc['gain_updates']} gain updates.\n",
         f"Twin calibrated to this run: K0 {K0c:.2f} °C/W (step tests {NOM.K0:.2f}), τ {tauc:.0f} s ({NOM.tau:.0f}), θ {thc:.0f} s; RMSE {rmse_cal:.2f} °C.\n",
         "| Controller | IAE total | Phase A: reach SP (min) | A: overshoot (°C) | A: settle ±0.5 (min) | Dist. 1: peak dev (°C) | Dist. 1: back in ±0.25 (min) | Dist. 2: peak dev (°C) | Dist. 2: back in ±0.25 (min) | Time in ±0.25 (%) | mean |Δu| (%/s) |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
for k, v in res.items():
    A, B, C = v["A"], v["B"], v["C"]
    lines.append(f"| {k} | {f(v['all']['IAE'])} | {f(A['t_reach_min'], 1)} | {A['overshoot']:.2f} | {f(A['recover_0.5'], 1)} | {B['max_dev']:.2f} | "
                 f"{f(B['recover_0.25'], 1)} | {C['max_dev']:.2f} | {f(C['recover_0.25'], 1)} | {v['all']['in025']:.0f} | {v['all']['mean_du']:.1f} |")
open(os.path.join(OUT, "stats.md"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))

# ---------------------------------------------------------------- 6. figures
plt.rcParams.update({"font.size": 10})
fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True, gridspec_kw=dict(height_ratios=[3, 1.3, 1.3]))
tm = grid / 60
for k, col in (("SIMC — twin", "#9aa0ad"), ("IMC — twin", "#c7b28a"), ("MATLAB pidtune (PI) — twin", "#d9a3a3")):
    ax[0].plot(tm, traj[k], color=col, lw=1.2, label=k.replace(" — twin", " (fixed gains, calibrated twin)"))
ax[0].plot(tm, Treal, color="#2e86de", lw=2.0, label="TD3 agent — REAL RIG")
ax[0].axhline(SP, color="k", ls="--", lw=0.9, label=f"setpoint {SP:.0f} °C")
ax[0].axhspan(SP - 0.25, SP + 0.25, color="#2e86de", alpha=0.07)
for tt, txt in ((T1, f"supply {Voc['A']:.1f} → {Voc['B']:.1f} V"), (T2, f"supply {Voc['B']:.1f} → {Voc['C']:.1f} V")):
    for x in ax:
        x.axvline(tt / 60, color="#e4572e", lw=1.2)
    ax[0].text(tt / 60 + 0.4, SP - 2.3, txt, color="#e4572e", fontsize=10, fontweight="bold")
ax[0].set_ylim(SP - 3, SP + 2.2); ax[0].set_ylabel("T [°C]"); ax[0].legend(loc="lower right", fontsize=8.5)
ax[0].set_title(f"Real rig, v2 agent: {SP:.0f} °C setpoint, two supply-voltage disturbances")
ax[1].plot(d.t_s / 60, d.PWM_pct, color="#2e86de", lw=0.8); ax[1].set_ylabel("PWM [%]")
for g, col in (("Kp", "#2e86de"), ("Ki", "#1f9d63"), ("Kd", "#e4572e")):
    ax[2].semilogy(d.t_s / 60, d[g], color=col, lw=1.2, label=g)
ax[2].set_ylabel("gains (log)"); ax[2].set_xlabel("time [min]"); ax[2].legend(loc="upper right", fontsize=8.5, ncol=3)
for x in ax:
    x.grid(alpha=.3)
plt.tight_layout(); plt.savefig(os.path.join(OUT, "overview.png"), dpi=120); plt.close()

names = [k for k in res if "oracle" not in k]
short = [k.replace(" — twin", "").replace("TD3 agent — REAL RIG", "TD3 agent (REAL RIG)").replace("TD3 agent", "TD3 agent (twin)")
         if k != "TD3 agent — REAL RIG" else "TD3 agent (REAL RIG)" for k in names]
fig, ax = plt.subplots(1, 3, figsize=(13, 4.6), sharey=True)
for j, (ph, ttl) in enumerate((("A", "Phase A: cold start (IAE)"), ("B", "Disturbance 1: supply ↑ (IAE)"), ("C", "Disturbance 2: supply ↓ (IAE)"))):
    vals = [res[k][ph]["IAE"] for k in names]
    cols = ["#2e86de" if "REAL" in k else ("#7fb3ea" if "TD3" in k else "#b8bcc6") for k in names]
    ax[j].barh(range(len(names)), vals, color=cols)
    for i, v in enumerate(vals):
        ax[j].text(v, i, f" {v:,.0f}", va="center", fontsize=8.5)
    ax[j].set_title(ttl, fontsize=10.5); ax[j].set_xlim(0, max(vals) * 1.25); ax[j].grid(axis="x", alpha=.3)
ax[0].set_yticks(range(len(names))); ax[0].set_yticklabels(short); ax[0].invert_yaxis()
plt.suptitle("Error by phase (IAE, °C·s; lower is better). Classical gains fixed, tuned once at 4 V. Grey/light blue = calibrated twin, blue = real rig", fontsize=10)
plt.tight_layout(); plt.savefig(os.path.join(OUT, "iae_by_phase.png"), dpi=120); plt.close()
print("wrote", OUT)
