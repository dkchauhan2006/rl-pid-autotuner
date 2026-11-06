"""
deploy_rl.py  —  runs the trained agent against the REAL rig over USB serial.

Arduino (arduino/pid_rig/pid_rig.ino) prints every second:
    T:<degC>,SP:<degC>,E:<degC>,U:<%>,V:<volts>,G:<Kp>,<Ki>,<Kd>
Every RL_EVERY seconds this script builds the SAME observation as the simulator, asks the policy for
gain changes and sends  K:<Kp>,<Ki>,<Kd>  back. The Arduino clamps the gains again (double safety).

    python deploy_rl.py COM3 --setpoint 50 --shadow     # shadow mode: logs what it WOULD send, sends nothing
    python deploy_rl.py COM3 --setpoint 50              # live tuning
    python deploy_rl.py COM3 --setpoint 50 --volts 4.0  # if the rig has no voltage-sense divider

Log: deploy_log.csv  (time, T, SP, U, V, Kp, Ki, Kd, sent)
"""
import argparse
import csv
import time

import numpy as np
import serial
from stable_baselines3 import TD3

from plant import PlantConfig
from thermal_pid_env import RL_EVERY, gain_to_z, z_to_gain, STEP_Z


def parse(line):
    try:
        parts = dict(kv.split(":", 1) for kv in line.strip().split(",") if ":" in kv)
        g = line.split("G:")[1].split(",")
        return dict(T=float(parts["T"]), SP=float(parts["SP"]), U=float(parts["U"]),
                    V=float(parts.get("V", "nan")), Kp=float(g[0]), Ki=float(g[1]), Kd=float(g[2]))
    except (KeyError, ValueError, IndexError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--model", default="models/td3_best/best_model.zip")
    ap.add_argument("--setpoint", type=float, required=True)
    ap.add_argument("--volts", type=float, default=None, help="fixed supply voltage if the rig cannot measure it")
    ap.add_argument("--shadow", action="store_true")
    ap.add_argument("--baud", type=int, default=9600)
    a = ap.parse_args()

    model = TD3.load(a.model, device="cpu")
    cfg = PlantConfig.nominal()
    ser = serial.Serial(a.port, a.baud, timeout=2)
    time.sleep(2)
    ser.write(f"S:{a.setpoint:.2f}\n".encode())
    print(f"{'SHADOW' if a.shadow else 'LIVE'} mode, setpoint {a.setpoint} °C. Ctrl+C to stop.")

    window, T_prev = [], None
    with open("deploy_log.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["time", "T", "SP", "U", "V", "Kp", "Ki", "Kd", "sent"])
        t0 = time.time()
        try:
            while True:
                d = parse(ser.readline().decode(errors="ignore"))
                if not d:
                    continue
                V = a.volts if a.volts is not None else (d["V"] if np.isfinite(d["V"]) and d["V"] > 0.5 else 4.0)
                window.append(d)
                sent = ""
                if len(window) >= RL_EVERY:
                    us = np.array([x["U"] for x in window])
                    T = d["T"]; T_prev = window[0]["T"] if T_prev is None else T_prev
                    z = np.array([gain_to_z(d["Kp"], "Kp"), gain_to_z(d["Ki"], "Ki"), gain_to_z(d["Kd"], "Kd")])
                    head = (d["SP"] - cfg.T_amb) / max(1e-3, cfg.T_max(V) - cfg.T_amb)
                    obs = np.array([(d["SP"] - T) / 10, np.clip(T - T_prev, -10, 10), us.mean() / 100 - .5,
                                    np.mean(np.abs(np.diff(us))) / 50 if len(us) > 1 else 0, *z,
                                    (V - 3.5) / 2, np.clip(head, 0, 2), (T - cfg.T_amb) / 50], dtype=np.float32)
                    act = model.predict(obs, deterministic=True)[0]
                    z = np.clip(z + STEP_Z * np.clip(act, -1, 1), -1, 1)
                    kp, ki, kd = z_to_gain(z[0], "Kp"), z_to_gain(z[1], "Ki"), z_to_gain(z[2], "Kd")
                    sent = f"K:{kp:.3f},{ki:.5f},{kd:.2f}"
                    if not a.shadow:
                        ser.write((sent + "\n").encode())
                    print(f"T={T:6.2f} SP={d['SP']:5.1f} U={d['U']:5.1f}% V={V:.2f}  ->  {sent}{'  (shadow)' if a.shadow else ''}")
                    T_prev, window = T, []
                w.writerow([round(time.time() - t0, 1), d["T"], d["SP"], d["U"], V, d["Kp"], d["Ki"], d["Kd"], sent])
                f.flush()
        except KeyboardInterrupt:
            print("stopped; last gains stay active on the Arduino")


if __name__ == "__main__":
    main()
