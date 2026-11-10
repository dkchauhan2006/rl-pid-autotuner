"""
system_id.py  —  Step 1 of the pipeline: real rig data -> plant model.

Input : data/raw/samplerun_2V.txt … samplerun_5V.txt (open-loop voltage steps, 5 thermocouples, 1 sample/s)
        + the lab notebook values (supply voltage, measured current, time the voltage was switched ON).
Output: models/plant_params.json  (used by the simulator, the classical tuning rules and the evaluation)
        results/system_id_fit.png (measured vs fitted curves, all channels, all voltages)

What it does
 1. Parses the CoolTerm logs (re-joins samples split over two lines, removes thermocouple glitches).
 2. Fits a FOPDT model per voltage and per channel:   T = T0 + dT_ss (1 - e^{-(t - t_on - θ)/τ})
 3. For the CONTROLLED channel (the farthest sensor, ch1) it fits one nonlinear model valid at every
    supply voltage, because heat loss grows with temperature (K drops from 7.9 to 6.8 °C/W):
        τ dT/dt = K0 · P(t - θ) - ΔT (1 + β ΔT)          P = (PWM%/100) · V² / R
    Steady state: K0 · P = ΔT (1 + β ΔT)

Run:  python system_id.py                  # downloads data/raw/*.txt from the GitHub repo (cached)
      python system_id.py --source local     # use the files already in data/raw/
"""
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
from datetime import datetime

import numpy as np
from scipy.ndimage import median_filter
from scipy.optimize import curve_fit, least_squares
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CONTROL_CHANNEL = 1          # 1-based. We track the FARTHEST sensor from the heater.
RUNS = {                     # from the lab notebook photo
    "2V": {"V": 2.0, "I": 0.51, "on": "17:12:00"},
    "3V": {"V": 3.0, "I": 0.81, "on": "22:44:00"},
    "4V": {"V": 4.0, "I": 1.08, "on": "08:56:27"},
    "5V": {"V": 5.0, "I": 1.38, "on": "20:22:04"},
}
TS = re.compile(r"^(\d{2}:\d{2}:\d{2})\t?(.*)$")
NUM = re.compile(r"-?\d+\.\d+|-?\d+")


# ----------------------------------------------------------------------------- parsing
def parse(path):
    """Return (t_seconds, temps[N,5], first_timestamp or None).
    CoolTerm sometimes splits one sample over two lines with the same timestamp ("29." + "5")."""
    lines = open(path, encoding="utf-8", errors="ignore").read().splitlines()
    rows, stamps, pre = [], [], []
    cur_ts, cur_txt = None, ""

    def flush():
        if cur_ts is not None:
            vals = [float(x) for x in NUM.findall(cur_txt)]
            if len(vals) == 5:
                rows.append(vals); stamps.append(cur_ts)

    for ln in lines:
        if not ln.strip():
            continue
        m = TS.match(ln)
        if m:
            ts, txt = m.group(1), m.group(2)
            if ts == cur_ts:
                cur_txt += txt
            else:
                flush(); cur_ts, cur_txt = ts, txt
        elif cur_ts is None:
            vals = [float(x) for x in NUM.findall(ln)]
            if len(vals) == 5:
                pre.append(vals)
    flush()

    if not stamps:                                   # no timestamps: 1 line = 1 s
        return np.arange(len(pre), dtype=float), np.array(pre, dtype=float), None
    t0 = datetime.strptime(stamps[0], "%H:%M:%S")
    t = np.array([(datetime.strptime(s, "%H:%M:%S") - t0).total_seconds() for s in stamps])
    t[t < 0] += 86400
    T = np.array(rows, dtype=float)
    if pre:
        t = np.concatenate([np.arange(-len(pre), 0, dtype=float), t])
        T = np.vstack([np.array(pre), T])
    return t, T, stamps[0]


def despike(T, win=9, thr=3.0):
    """A heater cannot move 3 °C in one second: anything that far from the local median is a glitch."""
    med = median_filter(T, size=(win, 1), mode="nearest")
    bad = np.abs(T - med) > thr
    out = T.copy(); out[bad] = med[bad]
    return out, int(bad.sum())


def onset_from_data(y, base_n=60, rise=0.5):
    base = np.median(y[:base_n])
    above = y > base + rise
    for i in range(len(y) - 30):
        if above[i:i + 30].all():
            return i
    return 0


def load_run(name, paths=None):
    info = RUNS[name]
    path = (paths or {}).get(name) or os.path.join(HERE, "data", "raw", f"samplerun_{name}.txt")
    t, T, first_ts = parse(path)
    T, n_spikes = despike(T)
    if first_ts:
        t_on = (datetime.strptime(info["on"], "%H:%M:%S") - datetime.strptime(first_ts, "%H:%M:%S")).total_seconds()
        theta_known = True
    else:                         # 2V/3V logs have no timestamps: detect the rise, dead time not identifiable
        t_on = float(t[onset_from_data(T.mean(axis=1))]); theta_known = False
    P = info["V"] * info["I"]
    return dict(name=name, t=t, T=T, t_on=t_on, theta_known=theta_known, P=P,
                V=info["V"], I=info["I"], R=info["V"] / info["I"], spikes=n_spikes)


# ----------------------------------------------------------------------------- per-run FOPDT
def fopdt(t, T0, dTss, tau, theta, t_on):
    out = np.full_like(t, T0, dtype=float)
    m = t >= t_on + theta
    out[m] = T0 + dTss * (1 - np.exp(-(t[m] - t_on - theta) / tau))
    return out


def fit_fopdt(t, y, t_on, theta_known):
    pre = y[t < t_on]
    T0 = float(np.median(pre[-60:])) if len(pre) > 5 else float(np.median(y[:30]))
    p0, lo, hi = [max(1.0, y[-1] - T0), 900.0, 20.0], [0.1, 10.0, 0.0], [300.0, 50000.0, 600.0]
    if not theta_known:
        p0[2], hi[2] = 0.0, 1e-6
    f = lambda tt, a, b, c: fopdt(tt, T0, a, b, c, t_on)
    popt, _ = curve_fit(f, t, y, p0=p0, bounds=(lo, hi), maxfev=20000)
    rmse = float(np.sqrt(np.mean((y - f(t, *popt)) ** 2)))
    return T0, popt, rmse


# ----------------------------------------------------------------------------- nonlinear global fit
def simulate_nl(t, t_on, T0, P, K0, beta, tau, theta):
    """Euler simulation of τ dΔT/dt = K0 P(t-θ) - ΔT(1+βΔT) for a constant power step at t_on."""
    dT, out = 0.0, np.empty_like(t)
    for i in range(len(t)):
        dt = (t[i] - t[i - 1]) if i else 0.0
        p = P if t[i] - t_on >= theta else 0.0
        dT += dt * (K0 * p - dT * (1 + beta * dT)) / tau
        out[i] = T0 + dT
    return out


def main(source="github", branch="main", refresh=False):
    paths = None
    if source == "github":
        from data_source import fetch_step_tests
        paths = fetch_step_tests(branch=branch, refresh=refresh)
        missing = [n for n in RUNS if n not in paths]
        if missing:
            raise SystemExit(f"step tests missing on GitHub: {missing}. Lab-note values exist for {list(RUNS)}.")
    runs = [load_run(n, paths) for n in RUNS]
    c = CONTROL_CHANNEL - 1
    per_run = {}
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))

    for ax, r in zip(axes.ravel(), runs):
        rows = []
        for ch in range(5):
            T0, (dTss, tau, theta), rmse = fit_fopdt(r["t"], r["T"][:, ch], r["t_on"], r["theta_known"])
            rows.append(dict(T0=T0, dTss=dTss, K=dTss / r["P"], tau=tau, theta=theta, rmse=rmse))
            ax.plot(r["t"] / 60, r["T"][:, ch], lw=1.6 if ch == c else 0.6, label=f"ch{ch+1}" + (" (controlled)" if ch == c else ""))
            ax.plot(r["t"] / 60, fopdt(r["t"], T0, dTss, tau, theta, r["t_on"]), "k--", lw=0.7)
        per_run[r["name"]] = rows
        ax.axvline(r["t_on"] / 60, color="r", lw=0.8)
        ax.set_title(f"{r['name']}   P = {r['P']:.2f} W   R = {r['R']:.2f} Ω")
        ax.set_xlabel("time [min]"); ax.set_ylabel("°C"); ax.grid(alpha=.3)
    axes[0, 0].legend(fontsize=7)
    plt.tight_layout(); os.makedirs(os.path.join(HERE, "results"), exist_ok=True); plt.savefig(os.path.join(HERE, "results", "system_id_fit.png"), dpi=130); plt.close()

    # ---- one nonlinear model for the controlled channel, fitted on all four runs together.
    # θ is only identifiable on the timestamped runs (4V, 5V); 2V/3V start at the detected rise.
    sub = []
    for r in runs:
        step = 3  # downsample for speed
        sub.append((r["t"][::step], r["T"][::step, c], r))

    def residuals(x):
        K0, beta, tau, theta = x
        res = []
        for t, y, r in sub:
            T0 = per_run[r["name"]][c]["T0"]
            th = theta if r["theta_known"] else 0.0
            res.append(simulate_nl(t, r["t_on"], T0, r["P"], K0, beta, tau, th) - y)
        return np.concatenate(res)

    sol = least_squares(residuals, x0=[8.0, 0.004, 900.0, 40.0],
                        bounds=([1.0, 0.0, 100.0, 0.0], [30.0, 0.05, 5000.0, 300.0]))
    K0, beta, tau, theta = sol.x
    rmse_nl = float(np.sqrt(np.mean(sol.fun ** 2)))

    ch_rows = [per_run[r["name"]][c] for r in runs]
    thetas = [row["theta"] for r, row in zip(runs, ch_rows) if r["theta_known"]]
    params = {
        "control_channel": CONTROL_CHANNEL,
        "note": "Farthest sensor from the heater. Model: tau*dT/dt = K0*P(t-theta) - dT*(1+beta*dT), P=(u/100)*V^2/R",
        "K0_C_per_W": round(float(K0), 4),
        "beta_per_C": round(float(beta), 6),
        "tau_s": round(float(tau), 1),
        "theta_s": round(float(theta), 1),
        "rmse_C": round(rmse_nl, 3),
        "R_ohm": {r["name"]: round(r["R"], 3) for r in runs},
        "R_ohm_mean": round(float(np.mean([r["R"] for r in runs])), 3),
        "T_ambient_C": round(float(np.mean([row["T0"] for row in ch_rows])), 2),
        # ranges for domain randomisation (observed spread + margin)
        "ranges": {
            "K0": [round(K0 * 0.9, 3), round(K0 * 1.1, 3)],
            "beta": [round(beta * 0.6, 6), round(beta * 1.4, 6)],
            # nonlinear-model tau (effective tau at a given temperature is tau/(1+2*beta*dT))
            "tau": [round(tau * 0.85, 1), round(tau * 1.15, 1)],
            "theta": [round(max(5.0, min(thetas) * 0.6), 1), round(max(thetas) * 1.3, 1)],
            "R": [round(min(r["R"] for r in runs) * 0.97, 3), round(max(r["R"] for r in runs) * 1.03, 3)],
            "V": [1.8, 5.5],
            "T_ambient": [26.0, 32.0],
        },
        "per_voltage_fopdt": {r["name"]: {"P_W": round(r["P"], 3), "K_C_per_W": round(row["K"], 3),
                                          "tau_s": round(row["tau"], 1), "theta_s": round(row["theta"], 1),
                                          "dT_ss": round(row["dTss"], 2), "rmse": round(row["rmse"], 3)}
                              for r, row in zip(runs, ch_rows)},
    }
    os.makedirs(os.path.join(HERE, "models"), exist_ok=True); json.dump(params, open(os.path.join(HERE, "models", "plant_params.json"), "w"), indent=2)

    print(f"Controlled channel: ch{CONTROL_CHANNEL} (farthest sensor)\n")
    print("Per-voltage FOPDT fit (controlled channel):")
    print("  run   P[W]   K[°C/W]  tau[s]  theta[s]  dT_ss  rmse")
    for r, row in zip(runs, ch_rows):
        th = f"{row['theta']:6.1f}" if r["theta_known"] else "   n/a"
        print(f"  {r['name']}  {r['P']:5.2f}   {row['K']:6.2f}  {row['tau']:6.0f}  {th}  {row['dTss']:5.1f}  {row['rmse']:.2f}")
    print(f"\nOne nonlinear model for all voltages: K0={K0:.3f} °C/W  beta={beta:.5f} 1/°C  tau={tau:.0f} s  "
          f"theta={theta:.1f} s  (rmse {rmse_nl:.2f} °C)")
    print("Saved models/plant_params.json and results/system_id_fit.png")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", choices=["github", "local"], default="github")
    ap.add_argument("--branch", default="main")
    ap.add_argument("--refresh", action="store_true", help="re-download the logs from GitHub")
    a = ap.parse_args()
    main(a.source, a.branch, a.refresh)
