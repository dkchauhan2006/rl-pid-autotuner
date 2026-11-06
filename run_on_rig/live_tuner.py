"""
live_tuner.py  —  run the trained RL agent on OUR rig (Arduino Nano R3 CH340) in real time.

Rig: B3950 100K NTC (A0, farthest point) · ACS712-30A current (A1) · 25V voltage sensor (A2) ·
     STP55NF06 MOSFET on D9 · 24 V 40 W ceramic cartridge heater.   Firmware: arduino_nano_rl_pid/arduino_nano_rl_pid.ino

Every second it reads one line from the Nano and prints it live:
    time | T | setpoint | error | PWM | V (A1) | I (A0) | P | Kp | Ki | Kd
Every 10 s it builds the same observation the agent saw in training, asks the TD3 policy for a gain change,
and sends  K:<Kp>,<Ki>,<Kd>  to the Nano (which applies it with SetTunings, without stopping the PID).

    python run_on_rig/live_tuner.py auto --setpoint 50 --shadow     # 1) watch only ("auto" finds the CH340 Nano)
    python run_on_rig/live_tuner.py COM5 --setpoint 50              # 2) live RL tuning
    python run_on_rig/live_tuner.py COM5 --setpoint 50 --fixed zn   # 3) comparison run with fixed ZN gains
    python run_on_rig/live_tuner.py COM5 --setpoint 50 --fixed matlab
    python run_on_rig/live_tuner.py COM5 --setpoint 50 --plot       # live plot window as well

Log: run_on_rig/logs/run_<mode>_<date-time>.csv   (open in Excel, or plot with --plot)
Stop: Ctrl+C. The Nano keeps the last gains and keeps controlling (safe by design).
"""
import argparse
import csv
import os
import sys
import time
from datetime import datetime

import numpy as np
import serial
from serial.tools import list_ports

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from plant import PlantConfig                                         # noqa: E402
from thermal_pid_env import RL_EVERY, STEP_Z, gain_to_z, z_to_gain    # noqa: E402
from tuning_rules import all_rules                                    # noqa: E402

MATLAB_4V = {"Kp": 4.69969225980263, "Ki": 0.00906372855301024, "Kd": 0.0}


def find_port():
    """Pick the Nano automatically: CH340 (our board), else FTDI/Arduino, else the only port present."""
    ports = list(list_ports.comports())
    for keys in (("CH340", "CH341", "USB-SERIAL"), ("FTDI", "ARDUINO", "USB SERIAL")):
        for p in ports:
            if any(k in f"{p.description} {p.manufacturer or ''}".upper() for k in keys):
                return p.device
    if len(ports) == 1:
        return ports[0].device
    raise SystemExit("Could not find the Nano. Ports: " + ", ".join(f"{p.device} ({p.description})" for p in ports)
                     + ". Install the CH340 driver or pass the port explicitly, e.g. COM5.")


def sanity_check(first):
    """Warn about wiring/calibration problems from the first seconds of data."""
    T = np.median([d["T"] for d in first]); V = np.median([d["V"] for d in first])
    I = np.median([d["I"] for d in first]); U = np.median([d["U"] for d in first])
    print(f"Sensor check: NTC(A0) {T:.1f} °C | voltage(A2) {V:.2f} V | current(A1) {I:.3f} A at PWM {U:.0f} %")
    if not (5 < T < 110):
        print("  ! A0 temperature looks wrong: check the NTC divider (R_FIXED, NTC_TO_GND) in the .ino")
    if V < 0.5:
        print("  ! A2 reads ~0 V: is the 25V sensor across the heater supply (before the MOSFET)? Using --volts or 4.0 V")
    if U > 50 and I < 0.05:
        print("  ! PWM is high but A1 current ~0: check the ACS712 wiring, or the MOSFET gate (STP55NF06 needs a strong 5 V drive)")


def parse(line):
    """T:41.25,SP:50.00,E:8.75,U:100.0,V:4.01,I:1.072,P:4.299,G:20.000,0.05000,100.00"""
    try:
        head, g = line.strip().split(",G:")
        d = {k: float(v) for k, v in (kv.split(":", 1) for kv in head.split(","))}
        kp, ki, kd = (float(x) for x in g.split(","))
        return dict(T=d["T"], SP=d["SP"], U=d["U"], V=d.get("V", float("nan")), I=d.get("I", float("nan")),
                    P=d.get("P", float("nan")), Kp=kp, Ki=ki, Kd=kd)
    except (ValueError, KeyError):
        return None


def build_obs(window, T_prev, V, cfg):
    """Exactly the 10-value observation of thermal_pid_env.ThermalPIDEnv._obs()."""
    d = window[-1]
    us = np.array([x["U"] for x in window])
    z = [gain_to_z(d["Kp"], "Kp"), gain_to_z(d["Ki"], "Ki"), gain_to_z(max(d["Kd"], 1.0), "Kd")]
    head = (d["SP"] - cfg.T_amb) / max(1e-3, cfg.T_max(V) - cfg.T_amb)
    return np.array([(d["SP"] - d["T"]) / 10.0,
                     np.clip(d["T"] - T_prev, -10, 10),
                     us.mean() / 100.0 - 0.5,
                     np.mean(np.abs(np.diff(us))) / 50.0 if len(us) > 1 else 0.0,
                     *z,
                     (V - 3.5) / 2.0,
                     np.clip(head, 0, 2),
                     (d["T"] - cfg.T_amb) / 50.0], dtype=np.float32), np.array(z)


def main():
    ap = argparse.ArgumentParser(description="RL PID auto-tuner, live on the Arduino Nano rig")
    ap.add_argument("port", help='"auto" (finds the CH340 Nano), or COM5 / /dev/ttyUSB0')
    ap.add_argument("--setpoint", type=float, required=True, help="target temperature of the farthest sensor, °C")
    ap.add_argument("--model", default=os.path.join(ROOT, "models", "td3_best", "best_model.zip"))
    ap.add_argument("--shadow", action="store_true", help="compute gains but do not send them")
    ap.add_argument("--fixed", choices=["zn", "simc", "matlab"], help="send fixed classical gains once (comparison run)")
    ap.add_argument("--volts", type=float, default=None, help="use this supply voltage if A1 is not wired")
    ap.add_argument("--plot", action="store_true", help="live matplotlib plot")
    ap.add_argument("--baud", type=int, default=9600)
    a = ap.parse_args()

    cfg = PlantConfig.nominal()
    model = None
    if not a.fixed:
        from stable_baselines3 import TD3
        model = TD3.load(a.model, device="cpu")
        print(f"Loaded agent: {os.path.relpath(a.model, ROOT)}")

    port = find_port() if a.port.lower() == "auto" else a.port
    print(f"Nano on {port}")
    ser = serial.Serial(port, a.baud, timeout=2)
    time.sleep(2)                                     # the Nano resets when the port opens
    ser.reset_input_buffer()
    ser.write(f"S:{a.setpoint:.2f}\n".encode())
    if a.fixed:
        sp = a.setpoint
        g = MATLAB_4V if a.fixed == "matlab" else all_rules(*cfg.fopdt_at(4.0, cfg.T_amb + 0.6 * (cfg.T_max(4.0) - cfg.T_amb)))[
            "ZN" if a.fixed == "zn" else "SIMC"]
        ser.write(f"K:{g['Kp']:.4f},{g['Ki']:.6f},{g['Kd']:.3f}\n".encode())
        print(f"Fixed {a.fixed.upper()} gains sent: Kp={g['Kp']:.3f} Ki={g['Ki']:.5f} Kd={g['Kd']:.2f}")
    mode = a.fixed or ("shadow" if a.shadow else "rl")

    os.makedirs(os.path.join(ROOT, "run_on_rig", "logs"), exist_ok=True)
    log_path = os.path.join(ROOT, "run_on_rig", "logs", f"run_{mode}_{datetime.now():%Y%m%d_%H%M%S}.csv")
    plot = None
    if a.plot:
        import matplotlib.pyplot as plt
        plt.ion(); fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        plot = (plt, fig, ax1, ax2, ax3)

    print(f"Mode: {mode.upper()} | setpoint {a.setpoint} °C | agent acts every {RL_EVERY} s | log: {os.path.relpath(log_path, ROOT)}")
    print(f"{'t[s]':>6} {'T[°C]':>7} {'SP':>6} {'err':>6} {'PWM%':>6} {'V':>5} {'I[A]':>6} {'P[W]':>6} | {'Kp':>8} {'Ki':>8} {'Kd':>8}  action")
    window, hist, T_prev, t0, first = [], [], None, time.time(), []
    with open(log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "T_C", "SP_C", "PWM_pct", "V", "I_A", "P_W", "Kp", "Ki", "Kd", "sent"])
        try:
            while True:
                d = parse(ser.readline().decode(errors="ignore"))
                if not d:
                    continue
                t = time.time() - t0
                if len(first) < 5:
                    first.append(d)
                    if len(first) == 5:
                        sanity_check(first)
                V = a.volts if a.volts else (d["V"] if np.isfinite(d["V"]) and d["V"] > 0.5 else 4.0)
                window.append(d)
                T_prev = d["T"] if T_prev is None else T_prev
                sent = ""
                if model is not None and len(window) >= RL_EVERY:
                    obs, z = build_obs(window, T_prev, V, cfg)
                    act = model.predict(obs, deterministic=True)[0]
                    z = np.clip(z + STEP_Z * np.clip(act, -1, 1), -1, 1)
                    kp, ki, kd = z_to_gain(z[0], "Kp"), z_to_gain(z[1], "Ki"), z_to_gain(z[2], "Kd")
                    sent = f"K:{kp:.3f},{ki:.5f},{kd:.2f}"
                    if not a.shadow:
                        ser.write((sent + "\n").encode())
                    T_prev, window = d["T"], []
                print(f"{t:6.0f} {d['T']:7.2f} {d['SP']:6.1f} {d['SP'] - d['T']:6.2f} {d['U']:6.1f} {V:5.2f} {d['I']:6.3f} {d['P']:6.2f} | "
                      f"{d['Kp']:8.3f} {d['Ki']:8.5f} {d['Kd']:8.2f}  {sent}{' (shadow, not sent)' if sent and a.shadow else ''}")
                w.writerow([round(t, 1), d["T"], d["SP"], d["U"], V, d["I"], d["P"], d["Kp"], d["Ki"], d["Kd"], sent]); f.flush()
                hist.append((t, d["T"], d["SP"], d["U"], d["Kp"], d["Ki"], d["Kd"]))
                if plot and len(hist) % 5 == 0:
                    plt, fig, ax1, ax2, ax3 = plot
                    h = np.array(hist)
                    for ax in (ax1, ax2, ax3): ax.cla(); ax.grid(alpha=.3)
                    ax1.plot(h[:, 0] / 60, h[:, 1], label="T"); ax1.plot(h[:, 0] / 60, h[:, 2], "k--", label="SP"); ax1.legend(); ax1.set_ylabel("°C")
                    ax2.plot(h[:, 0] / 60, h[:, 3], lw=.7); ax2.set_ylabel("PWM %")
                    for i, n in enumerate(["Kp", "Ki", "Kd"]): ax3.semilogy(h[:, 0] / 60, np.maximum(h[:, 4 + i], 1e-4), label=n)
                    ax3.legend(); ax3.set_ylabel("gains"); ax3.set_xlabel("min"); plt.pause(0.01)
        except KeyboardInterrupt:
            print(f"\nStopped. The Nano keeps the last gains. Log saved: {log_path}")


if __name__ == "__main__":
    main()
