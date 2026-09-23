"""
fit_fopdt.py

THIS is where your real experimental data is used -- not to train the RL
agent directly, but to figure out the realistic K/tau/theta RANGE that the
simulator (thermal_pid_env.py) should randomize over.

Required input file: step_test_data.csv  (see log_step_test.py to generate it)
Exact expected format (3 columns, header row required):

    time_s,u_percent,temp_C
    0.0,0,25.0
    0.5,0,25.1
    1.0,50,25.1
    1.5,50,25.4
    ...

How to collect this data on the real rig:
  1. Let the system sit at steady state (constant temp) with heater OFF (u=0).
  2. At t=0, suddenly set heater output to a fixed value, e.g. u=50%, and hold it.
  3. Log time, u, and temperature every 0.5-1s for several minutes until temp
     plateaus (reaches new steady state).
  4. Repeat 2-3 times at different step sizes (e.g. 30%, 50%, 70%) for a more
     robust fit -- concatenate them with a time offset, or fit separately and
     average the resulting K/tau/theta.

Run:
    python fit_fopdt.py
"""

import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CSV_PATH = "step_test_data.csv"


def fopdt_response(t, K, tau, theta, T0, u_step):
    T_pred = np.empty_like(t)
    mask = t < theta
    T_pred[mask] = T0
    T_pred[~mask] = T0 + K * u_step * (1 - np.exp(-(t[~mask] - theta) / tau))
    return T_pred


def main():
    df = pd.read_csv(CSV_PATH)
    t = df["time_s"].to_numpy(dtype=float)
    u = df["u_percent"].to_numpy(dtype=float)
    T = df["temp_C"].to_numpy(dtype=float)

    T0 = T[0]
    u_step = u[-1] - u[0]
    if abs(u_step) < 1e-6:
        raise ValueError("u_percent doesn't change in this file -- need a real step input.")

    def model(t, K, tau, theta):
        return fopdt_response(t, K, tau, theta, T0, u_step)

    p0 = [2.0, 60.0, 2.0]  # initial guess: K, tau, theta
    bounds = ([0.01, 1.0, 0.0], [50.0, 1000.0, 120.0])
    popt, _ = curve_fit(model, t, T, p0=p0, bounds=bounds, maxfev=10000)
    K, tau, theta = popt

    print("=" * 50)
    print(f"Fitted FOPDT parameters:")
    print(f"  K (process gain)   = {K:.4f}  [deg C per % heater output]")
    print(f"  tau (time constant) = {tau:.2f} s")
    print(f"  theta (dead time)   = {theta:.2f} s")
    print("=" * 50)

    # +/-30% domain-randomization range for thermal_pid_env.py
    print("\nPaste these into thermal_pid_env.py (ThermalPIDEnv defaults):")
    print(f"    K_range=({K*0.7:.3f}, {K*1.3:.3f}),")
    print(f"    tau_range=({tau*0.7:.1f}, {tau*1.3:.1f}),")
    print(f"    theta_range=({max(0.2, theta*0.5):.1f}, {theta*1.5 + 0.5:.1f}),")

    plt.figure(figsize=(8, 5))
    plt.plot(t, T, "o", markersize=3, label="measured (real rig)")
    plt.plot(t, model(t, *popt), "-", linewidth=2, label="fitted FOPDT")
    plt.xlabel("time (s)")
    plt.ylabel("Temperature (deg C)")
    plt.title("FOPDT fit to step-test data")
    plt.legend()
    plt.grid(alpha=0.3)
    plt.savefig("fopdt_fit.png", dpi=150)
    print("\nSaved fit plot to fopdt_fit.png -- check it visually before trusting the numbers.")


if __name__ == "__main__":
    main()
