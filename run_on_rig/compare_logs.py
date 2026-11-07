"""
compare_logs.py  —  real-rig A/B comparison: RL agent vs fixed classical gains.

Do two (or more) runs on the rig with the SAME setpoint and starting from the SAME temperature (let it cool to
room temperature in between), e.g.
    python run_on_rig/live_tuner.py COM8 --setpoint 50              # RL agent      -> logs/run_rl_*.csv
    python run_on_rig/live_tuner.py COM8 --setpoint 50 --fixed simc # best classical -> logs/run_simc_*.csv
then
    python run_on_rig/compare_logs.py                      # compares the newest RL log with the newest fixed-gain log
    python run_on_rig/compare_logs.py logs/run_rl_X.csv logs/run_simc_Y.csv logs/run_zn_Z.csv

Prints IAE, overshoot, settling time, steady-state error and PWM chatter for each run, and saves a plot
(run_on_rig/logs/comparison.png). Runs are cut to the length of the shortest run so the comparison is fair.
"""
import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, "logs")


def load(path):
    import csv
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path} has no data rows")
    g = lambda k: np.array([float(r[k]) for r in rows])
    return dict(t=g("t_s"), T=g("T_C"), SP=g("SP_C"), U=g("PWM_pct"), name=os.path.basename(path))


def metrics(r, n):
    t, T, SP, U = r["t"][:n] - r["t"][0], r["T"][:n], r["SP"][:n], r["U"][:n]
    e = SP - T
    step = float(np.median(np.diff(t))) if n > 1 else 1.0
    dt = step if step > 0.1 else 1.0                      # the Nano sends one line per second
    if step <= 0.1:                                       # timestamps unusable -> 1 s per sample
        t = np.arange(n, dtype=float)
    span = abs(SP[-1] - T[0]) or 1.0
    over = max(0.0, (np.max(T) - SP[-1]) / span * 100) if SP[-1] >= T[0] else max(0.0, (SP[-1] - np.min(T)) / span * 100)
    tol = max(0.02 * span, 0.5)
    out = np.where(np.abs(e) > tol)[0]
    settle = "never" if len(out) and out[-1] == n - 1 else f"{(t[out[-1]] if len(out) else 0):.0f}"
    tail = max(1, min(n // 5, 300))
    return dict(IAE=float(np.sum(np.abs(e)) * dt), overshoot=over, settling=settle,
                ss_err=float(np.mean(np.abs(e[-tail:]))), chatter=float(np.mean(np.abs(np.diff(U)))) if n > 1 else 0.0,
                duration=float(t[-1]))


def main():
    paths = sys.argv[1:]
    if not paths:
        rl = sorted(glob.glob(os.path.join(LOGS, "run_rl_*.csv")), key=os.path.getmtime)
        fx = sorted([p for p in glob.glob(os.path.join(LOGS, "run_*.csv"))
                     if os.path.basename(p).split("_")[1] in ("simc", "zn", "matlab")], key=os.path.getmtime)
        if not rl or not fx:
            raise SystemExit("Need at least one run_rl_*.csv and one run_simc/zn/matlab_*.csv in run_on_rig/logs/")
        paths = [rl[-1], fx[-1]]
    runs = [load(p) for p in paths]
    n = min(len(r["t"]) for r in runs)
    print(f"Comparing {len(runs)} runs over the first {n} samples (~{n/60:.0f} min each)\n")
    print(f"{'run':38s} {'IAE °C·s':>10} {'overshoot %':>12} {'settling s':>11} {'ss err °C':>10} {'chatter %/s':>12}")
    ms = []
    for r in runs:
        m = metrics(r, n); ms.append(m)
        print(f"{r['name']:38s} {m['IAE']:10.0f} {m['overshoot']:12.1f} {m['settling']:>11} {m['ss_err']:10.2f} {m['chatter']:12.1f}")
    best = min(range(len(ms)), key=lambda i: ms[i]["IAE"])
    print(f"\nLowest IAE: {runs[best]['name']}")
    if len(runs) >= 2:
        a, b = ms[0], ms[1]
        print(f"Accuracy of {runs[0]['name']} relative to {runs[1]['name']}: {b['IAE'] / a['IAE'] * 100:.1f} % "
              f"(IAE ratio; >100 % = first run better). Chatter {a['chatter']:.1f} vs {b['chatter']:.1f} %/s")

    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for r in runs:
        tt = (r["t"][:n] - r["t"][0]) / 60
        ax[0].plot(tt, r["T"][:n], label=r["name"]); ax[1].plot(tt, r["U"][:n], lw=0.6, label=r["name"])
    ax[0].plot((runs[0]["t"][:n] - runs[0]["t"][0]) / 60, runs[0]["SP"][:n], "k--", label="setpoint")
    ax[0].set_ylabel("T [°C]"); ax[1].set_ylabel("PWM [%]"); ax[1].set_xlabel("time [min]")
    for x in ax: x.grid(alpha=.3); x.legend(fontsize=8)
    ax[0].set_title("Real rig: RL agent vs fixed classical gains")
    out = os.path.join(LOGS, "comparison.png")
    plt.tight_layout(); plt.savefig(out, dpi=120)
    print(f"Plot saved: {out}")


if __name__ == "__main__":
    main()
