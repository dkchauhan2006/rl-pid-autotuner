"""
evaluate.py  —  RL agent vs classical tuning (ZN, Cohen-Coon, SIMC, AMIGO, IMC, optional MATLAB pidtune).

Every controller runs on the SAME digital twin, SAME scenario, SAME sensor-noise seed.
Classical gains are tuned once at the nominal operating point (4 V, 60 % of the reachable rise),
like a real commissioning. "ZN re-tuned" is an oracle that is re-tuned for the exact voltage
(it knows the plant; a real rig would need a new step test for that).

    python evaluate.py                         # uses models/td3_best/best_model.zip if present
    python evaluate.py --model models/td3_pid_tuner.zip --random 40

Writes results/summary.md and results/*.png
"""
import argparse
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from plant import PlantConfig, load_params, step_metrics
from thermal_pid_env import ThermalPIDEnv
from tuning_rules import all_rules

V_TUNE, HEAD_TUNE = 4.0, 0.6
# Paste gains from matlab/pidtune_compare.m here for an exact MATLAB comparison, e.g.
# BASELINE_OVERRIDES = {"MATLAB pidtune": {"Kp": 40.1, "Ki": 0.11, "Kd": 300.0}}
BASELINE_OVERRIDES = {}


def classical_gains(cfg):
    sp = cfg.T_amb + HEAD_TUNE * (cfg.T_max(V_TUNE) - cfg.T_amb)
    g = all_rules(*cfg.fopdt_at(V_TUNE, sp))
    g.update(BASELINE_OVERRIDES)
    return g


def scenario(cfg, name):
    sp = lambda V, h: cfg.T_amb + h * (cfg.T_max(V) - cfg.T_amb)
    S = {
        "cold_2V": dict(V1=2.0, V2=2.0, sp1=sp(2.0, .6), sp2=sp(2.0, .6), T0=cfg.T_amb, t_event=10**9),
        "cold_3V": dict(V1=3.0, V2=3.0, sp1=sp(3.0, .6), sp2=sp(3.0, .6), T0=cfg.T_amb, t_event=10**9),
        "cold_4V": dict(V1=4.0, V2=4.0, sp1=sp(4.0, .6), sp2=sp(4.0, .6), T0=cfg.T_amb, t_event=10**9),
        "cold_5V": dict(V1=5.0, V2=5.0, sp1=sp(5.0, .6), sp2=sp(5.0, .6), T0=cfg.T_amb, t_event=10**9),
        "supply_drop_5to3V": dict(V1=5.0, V2=3.0, sp1=sp(3.0, .75), sp2=sp(3.0, .75), T0=sp(3.0, .75), t_event=900),
        "supply_rise_3to5V": dict(V1=3.0, V2=5.0, sp1=sp(3.0, .75), sp2=sp(3.0, .75), T0=sp(3.0, .75), t_event=900),
        "sp_step_4V": dict(V1=4.0, V2=4.0, sp1=sp(4.0, .4), sp2=sp(4.0, .75), T0=sp(4.0, .4), t_event=900),
    }[name]
    S.update(cfg=cfg, kind=name)
    return S


def run(sc, gains=None, model=None, seed=0):
    """Run one episode. gains: fixed classical gains. model: RL policy (starts from `gains` if given,
    else from deliberately bad gains)."""
    init = gains or {"Kp": 3.0, "Ki": 0.005, "Kd": 5.0}
    env = ThermalPIDEnv(randomize=False, scenario=sc, seed=seed, init_gains=init)
    obs, _ = env.reset(seed=seed)
    done = False
    while not done:
        a = model.predict(obs, deterministic=True)[0] if model is not None else np.zeros(3)
        obs, _, te, tr, _ = env.step(a)
        done = te or tr
    return env.trajectory()


def metrics(traj, sc):
    t, T, SP, u = traj[:, 0], traj[:, 1], traj[:, 2], traj[:, 3]
    t_ref = 0 if sc["t_event"] > t[-1] else sc["t_event"]
    m = step_metrics(t, T, SP, t_ref)
    m["chatter_%/s"] = float(np.mean(np.abs(np.diff(u))))
    m["IAE_total"] = float(np.sum(np.abs(SP - T)))
    after = t >= t_ref
    m["max_dev_C"] = float(np.max(np.abs(SP[after] - T[after])))
    # overshoot % only makes sense for a real setpoint step (not for a supply-voltage disturbance)
    m["sp_step"] = bool(abs(SP[after][-1] - T[after][0]) >= 2.0)
    return m


def fmt_s(x):
    return "never" if not np.isfinite(x) else f"{x:.0f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=None)
    ap.add_argument("--random", type=int, default=30, help="number of random plants for the robustness table")
    a = ap.parse_args()

    path = a.model or next((p for p in ["models/td3_best/best_model.zip", "models/td3_pid_tuner.zip"] if os.path.exists(p)), None)
    model = None
    if path:
        from stable_baselines3 import TD3, DDPG
        model = (DDPG if "ddpg" in path.lower() else TD3).load(path, device="cpu")
        print(f"RL model: {path}")
    else:
        print("No trained model found: showing classical baselines only.")

    os.makedirs("results", exist_ok=True)
    params = load_params()
    cfg = PlantConfig.nominal(params)
    base = classical_gains(cfg)
    lines = ["# Results: RL agent vs classical PID tuning", "",
             f"Plant: digital twin of the rig, farthest sensor (ch{params['control_channel']}). "
             f"Classical gains tuned once at {V_TUNE:.0f} V. Same plant, scenario and noise seed for every controller.", "",
             "## Classical gains (tuned at 4 V)", "", "| rule | Kp | Ki | Kd |", "|---|---|---|---|"]
    lines += [f"| {n} | {g['Kp']:.2f} | {g['Ki']:.4f} | {g['Kd']:.1f} |" for n, g in base.items()]

    names = ["cold_2V", "cold_3V", "cold_4V", "cold_5V", "supply_drop_5to3V", "supply_rise_3to5V", "sp_step_4V"]
    all_rows = {}
    for sn in names:
        sc = scenario(cfg, sn)
        rows, trajs = {}, {}
        for n, g in base.items():
            trajs[n] = run(sc, g); rows[n] = metrics(trajs[n], sc)
        Vref = sc["V1"]
        g_or = all_rules(*cfg.fopdt_at(Vref, sc["sp2"]))["ZN"]
        trajs["ZN re-tuned (oracle)"] = run(sc, g_or); rows["ZN re-tuned (oracle)"] = metrics(trajs["ZN re-tuned (oracle)"], sc)
        if model is not None:
            trajs["RL (from bad gains)"] = run(sc, None, model); rows["RL (from bad gains)"] = metrics(trajs["RL (from bad gains)"], sc)
            trajs["RL (from ZN gains)"] = run(sc, base["ZN"], model); rows["RL (from ZN gains)"] = metrics(trajs["RL (from ZN gains)"], sc)
        all_rows[sn] = rows

        step = next(iter(rows.values()))["sp_step"]
        col = "overshoot %" if step else "max deviation °C"
        lines += ["", f"## {sn}", "", f"| controller | IAE (°C·s) | {col} | settling s | ss error °C | PWM chatter %/s |", "|---|---|---|---|---|---|"]
        for n, m in rows.items():
            ov = f"{m['overshoot_%']:.1f}" if step else f"{m['max_dev_C']:.2f}"
            lines.append(f"| {n} | {m['IAE']:.0f} | {ov} | {fmt_s(m['settling_s'])} | {m['ss_error']:.2f} | {m['chatter_%/s']:.1f} |")

        # plot: temperature + PWM (+ RL gains)
        show = [k for k in ["ZN", "SIMC", "ZN re-tuned (oracle)", "RL (from bad gains)"] if k in trajs]
        fig, ax = plt.subplots(3 if model else 2, 1, figsize=(11, 9 if model else 6.5), sharex=True)
        for k in show:
            tr = trajs[k]; ax[0].plot(tr[:, 0] / 60, tr[:, 1], lw=1.2, label=k)
        tr = trajs[show[0]]
        ax[0].plot(tr[:, 0] / 60, tr[:, 2], "k--", lw=1, label="setpoint"); ax[0].set_ylabel("T [°C]"); ax[0].legend(fontsize=8)
        for k in [s for s in ["ZN", "RL (from bad gains)"] if s in trajs]:
            ax[1].plot(trajs[k][:, 0] / 60, trajs[k][:, 3], lw=0.6, label=k)
        ax[1].set_ylabel("PWM [%]"); ax[1].legend(fontsize=8)
        if model:
            g = trajs["RL (from bad gains)"]
            for i, n in enumerate(["Kp", "Ki", "Kd"]):
                ax[2].semilogy(g[:, 0] / 60, g[:, 5 + i], label=f"RL {n}")
            ax[2].set_ylabel("gains (log)"); ax[2].legend(fontsize=8)
        if sc["t_event"] < 10**8:
            for x in ax: x.axvline(sc["t_event"] / 60, color="r", lw=0.7)
        ax[-1].set_xlabel("time [min]"); ax[0].set_title(sn); [x.grid(alpha=.3) for x in ax]
        plt.tight_layout(); plt.savefig(f"results/{sn}.png", dpi=110); plt.close()
        print(f"{sn}: " + " | ".join(f"{n}: IAE {m['IAE']:.0f}, chat {m['chatter_%/s']:.1f}" for n, m in rows.items()))

    # robustness over random plants + random scenarios (drift, other voltages)
    if a.random > 0:
        rng = np.random.default_rng(123)
        agg = {}
        for i in range(a.random):
            env = ThermalPIDEnv(seed=1000 + i)
            sc = env._sample_scenario()
            g_fixed = classical_gains(PlantConfig.nominal(params))   # commissioning on the nominal model
            ctrls = {n: (g, None) for n, g in g_fixed.items()}
            if model is not None:
                ctrls["RL (from bad gains)"] = (None, model)
                ctrls["RL (from ZN gains)"] = (g_fixed["ZN"], model)
            for n, (g, mdl) in ctrls.items():
                tr = run(sc, g, mdl, seed=2000 + i)
                agg.setdefault(n, []).append(metrics(tr, sc))
        lines += ["", f"## Robustness: {a.random} random plants (K0, β, τ, θ, R, ambient, noise) and random scenarios", "",
                  "| controller | mean IAE (°C·s) | median IAE | runs not settled at the end | mean PWM chatter %/s |", "|---|---|---|---|---|"]
        for n, ms in agg.items():
            iae = [m['IAE_total'] for m in ms]
            lines.append(f"| {n} | {np.mean(iae):.0f} | {np.median(iae):.0f} | "
                         f"{sum(1 for m in ms if not np.isfinite(m['settling_s']))}/{len(ms)} | {np.mean([m['chatter_%/s'] for m in ms]):.1f} |")
        print("robustness done")

    open("results/summary.md", "w", encoding="utf-8").write("\n".join(lines) + "\n")
    json.dump({k: v for k, v in all_rows.items()}, open("results/metrics.json", "w"), indent=1, default=float)
    print("wrote results/summary.md, results/metrics.json and results/*.png")


if __name__ == "__main__":
    main()
