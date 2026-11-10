"""
tools/check_v2_agent.py  —  quick check of the trained agent on the rig's real test case
(4 V supply, setpoint 40 °C, cold start), starting from the firmware's default gains (20, 0.05, 100),
from SIMC gains, and from 4x-wrong gains. Prints IAE vs fixed SIMC and saves results/v2_sp40_check.png.

    python tools/check_v2_agent.py [--model models/td3_best/best_model.zip]
"""
import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)
from eval_suite import STARTS, run_episode, scenarios      # noqa: E402
from plant import PlantConfig, load_params                  # noqa: E402
from thermal_pid_env import base_gains, z_from_gains        # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--model", default="models/td3_best/best_model.zip")
a = ap.parse_args()
from stable_baselines3 import TD3                            # noqa: E402
model = TD3.load(a.model, device="cpu")
pol = lambda o: model.predict(o, deterministic=True)[0]      # noqa: E731

cfg = PlantConfig.nominal(load_params())
sc = scenarios(cfg)["cold_4V_SP40"]
z_fw = z_from_gains({"Kp": 20.0, "Ki": 0.05, "Kd": 100.0}, base_gains(cfg, 4.0, 40.0))
cases = {"firmware defaults": z_fw, **{f"{k} start": v for k, v in STARTS.items()}}
simc_m, simc_tr = run_episode(sc, STARTS["SIMC"])
print(f"fixed SIMC: IAE {simc_m['IAE']:.0f}, overshoot {simc_m['overshoot']:.2f} C, ss err {simc_m['ss_err']:.3f} C")
fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
ax[0].plot(simc_tr[:, 0] / 60, simc_tr[:, 1], "k", lw=1.5, label="fixed SIMC")
for name, z0 in cases.items():
    m, tr = run_episode(sc, z0, pol)
    fixed, _ = run_episode(sc, z0)
    print(f"RL from {name:18s}: IAE {m['IAE']:6.0f} ({m['IAE'] / simc_m['IAE']:.2f}x SIMC; same gains fixed: "
          f"{fixed['IAE'] / simc_m['IAE']:.2f}x) overshoot {m['overshoot']:.2f} C  ss err {m['ss_err']:.3f} C  "
          f"final Kp {m['Kp_end']:.1f} Ki {m['Ki_end']:.4f} Kd {m['Kd_end']:.0f}")
    ax[0].plot(tr[:, 0] / 60, tr[:, 1], lw=1, label=f"RL from {name}")
    ax[1].semilogy(tr[:, 0] / 60, tr[:, 5], lw=1, label=f"Kp ({name})")
    ax[2].semilogy(tr[:, 0] / 60, tr[:, 6], lw=1, label=f"Ki ({name})")
ax[0].axhline(40, color="k", ls="--", lw=0.8); ax[0].set_ylabel("T [°C]"); ax[0].set_ylim(28, 44); ax[0].legend(fontsize=8)
ax[1].set_ylabel("Kp"); ax[2].set_ylabel("Ki"); ax[2].set_xlabel("min")
for x in ax:
    x.grid(alpha=.3)
ax[0].set_title("v2 agent, digital twin: 4 V, setpoint 40 °C, cold start")
plt.tight_layout(); os.makedirs("results", exist_ok=True); plt.savefig("results/v2_sp40_check.png", dpi=110)
print("saved results/v2_sp40_check.png")
