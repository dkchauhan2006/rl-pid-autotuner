"""
live_tuner.py  —  run the trained RL agent on OUR rig (Arduino Nano R3 CH340) in real time.

Rig: B3950 100K NTC (A0, farthest point) · ACS712-30A current (A1) · 25V voltage sensor (A2) ·
     STP55NF06 MOSFET on D6 (100 Ω gate resistor) · ceramic cartridge heater.   Firmware: arduino_nano_rl_pid/arduino_nano_rl_pid.ino

Every second it reads one line from the Nano and prints it live:
    time | T | setpoint | error | PWM | V (A1) | I (A0) | P | Kp | Ki | Kd
Every 10 s it builds the same observation the agent saw in training, asks the TD3 policy for a gain change,
and sends  K:<Kp>,<Ki>,<Kd>  to the Nano (which applies it with SetTunings, without stopping the PID).

    python run_on_rig/live_tuner.py COM8 --setpoint 50 --shadow     # 1) watch only ("auto" finds the CH340 Nano)
    python run_on_rig/live_tuner.py COM5 --setpoint 50              # 2) live RL tuning
    python run_on_rig/live_tuner.py COM5 --setpoint 50 --fixed zn   # 3) comparison run with fixed ZN gains
    python run_on_rig/live_tuner.py COM5 --setpoint 50 --fixed matlab
    python run_on_rig/live_tuner.py COM8 --setpoint 50 --plot       # live plot window as well
    python run_on_rig/live_tuner.py COM8 --setpoint 50 --compare simc   # RL on the rig + SIMC on the twin, side by side

Heater: our step tests (2–5 V supply, as our professor asked) measured R = V/I ≈ 3.7 Ω, so that is the default.
The script also measures the heater resistance live (R = V·duty / I from A2 and A1) and shows the agent an EQUIVALENT
voltage  V_eq = V·sqrt(R_model / R_heater), i.e. the voltage that gives the same heater power on the trained model.
With the ~3.7 Ω heater V_eq ≈ V, and the trained range is 1.8–5.5 V, so run the supply at 2–5 V.
If the USB link drops (it can when the heater switches), the script reconnects automatically.

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
from thermal_pid_env import RL_EVERY, STEP_Z, base_gains, gains_from_z, z_from_gains    # noqa: E402
from tuning_rules import all_rules                                    # noqa: E402
from plant import Plant, PID                                          # noqa: E402

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


V_TRAIN = (1.8, 5.5)          # supply range of the training scenarios (on the ~3.7 Ω model heater)


def supply_range(R_heater, cfg):
    """Supply voltages that keep heater power inside the trained range, for this heater."""
    k = np.sqrt(R_heater / cfg.R)
    return V_TRAIN[0] * k, V_TRAIN[1] * k


def sanity_check(first):
    """Warn about wiring/calibration problems from the first seconds of data."""
    T = np.median([d["T"] for d in first]); V = np.median([d["V"] for d in first])
    I = np.median([d["I"] for d in first]); U = np.median([d["U"] for d in first])
    print(f"Sensor check: NTC(A0) {T:.1f} °C | voltage(A2) {V:.2f} V | current(A1) {I:.3f} A at PWM {U:.0f} %")
    if not (5 < T < 100):
        print("  ! A0 temperature looks wrong: check the NTC divider (R_FIXED, NTC_TO_GND) in the .ino")
        A = np.median([d["A"] for d in first])
        if np.isfinite(A) and 2 < A < 1021:
            ratio = A / 1023.0
            Tof = lambda r: 1.0 / (1.0 / 298.15 + np.log(r / 100e3) / 3950.0) - 273.15
            print(f"    raw A0 = {A:.0f} ({A * 5 / 1023:.2f} V). Same reading interpreted with other dividers:")
            for Rf in (4.7e3, 10e3, 100e3):
                print(f"      R_FIXED {Rf / 1e3:>5g} kΩ: NTC to GND -> {Tof(Rf * ratio / (1 - ratio)):6.1f} °C | "
                      f"NTC to 5V -> {Tof(Rf * (1 - ratio) / ratio):6.1f} °C")
            print("    Pick the one that matches room temperature and set R_FIXED / NTC_TO_GND in the .ino, then re-upload.")
            print("    Check that the NTC is really on A0: warm its tip with your fingers, A must change.")
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
                    P=d.get("P", float("nan")), A=d.get("A", float("nan")), Kp=kp, Ki=ki, Kd=kd)
    except (ValueError, KeyError):
        return None


def build_obs(window, T_prev, V, cfg):
    """Exactly the 10-value observation of thermal_pid_env.ThermalPIDEnv._obs().
    Gains are seen as multipliers of the model-based SIMC gains at this voltage and setpoint (v2 agent)."""
    d = window[-1]
    us = np.array([x["U"] for x in window])
    base = base_gains(cfg, V, d["SP"])
    z = z_from_gains((d["Kp"], d["Ki"], d["Kd"]), base)
    head = (d["SP"] - cfg.T_amb) / max(1e-3, cfg.T_max(V) - cfg.T_amb)
    return np.array([(d["SP"] - d["T"]) / 10.0,
                     np.clip(d["T"] - T_prev, -10, 10),
                     us.mean() / 100.0 - 0.5,
                     np.mean(np.abs(np.diff(us))) / 50.0 if len(us) > 1 else 0.0,
                     *z,
                     (V - 3.5) / 2.0,
                     np.clip(head, 0, 2),
                     (d["T"] - cfg.T_amb) / 50.0], dtype=np.float32), np.array(z), base


def main():
    ap = argparse.ArgumentParser(description="RL PID auto-tuner, live on the Arduino Nano rig")
    ap.add_argument("port", help='"auto" (finds the CH340 Nano), or COM5 / /dev/ttyUSB0')
    ap.add_argument("--setpoint", type=float, required=True, help="target temperature of the farthest sensor, °C")
    ap.add_argument("--model", default=os.path.join(ROOT, "models", "td3_best", "best_model.zip"))
    ap.add_argument("--shadow", action="store_true", help="compute gains but do not send them")
    ap.add_argument("--fixed", choices=["zn", "simc", "matlab"], help="send fixed classical gains once (comparison run)")
    ap.add_argument("--volts", type=float, default=None, help="use this supply voltage if A2 is not wired")
    ap.add_argument("--ambient", type=float, default=None,
                    help="room temperature in °C (default: the first readings, if the rig starts cold)")
    ap.add_argument("--heater-ohms", type=float, default=None,
                    help="heater resistance until it is measured on the rig (default: the 3.7 Ω measured in our step tests)")
    ap.add_argument("--plot", action="store_true", help="live matplotlib plot")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--raw", action="store_true", help="also print every raw line received from the Nano (debugging)")
    ap.add_argument("--compare", choices=["simc", "zn", "matlab"], default=None,
                    help="run this classical tuning on the digital twin in parallel (same setpoint and voltage) and show both")
    a = ap.parse_args()

    cfg = PlantConfig.nominal()

    def classical(name):
        if name == "matlab":
            return MATLAB_4V
        sp4 = cfg.T_amb + 0.6 * (cfg.T_max(4.0) - cfg.T_amb)
        return all_rules(*cfg.fopdt_at(4.0, sp4))["ZN" if name == "zn" else "SIMC"]

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
        g = classical(a.fixed)
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
    R_heater = a.heater_ohms or cfg.R
    lo, hi = supply_range(R_heater, cfg)
    print(f"Heater: assuming {R_heater:.1f} Ω until measured -> keep the supply within {lo:.1f}–{hi:.1f} V (trained power range)")
    twin = twin_pid = None
    iae_real = iae_twin = 0.0
    if a.compare:
        gc = classical(a.compare)
        twin_pid = PID(gc["Kp"], gc["Ki"], max(gc["Kd"], 0.0))
        print(f"Compare: {a.compare.upper()} gains Kp={gc['Kp']:.3f} Ki={gc['Ki']:.5f} Kd={gc['Kd']:.2f} run on the DIGITAL TWIN "
              f"with the same setpoint and voltage (columns T_{a.compare} and IAE). For a real-rig A/B test use --fixed {a.compare} + compare_logs.py")
    cmp_hdr = f" {'T_' + a.compare:>7} {'IAE_rl':>7} {'IAE_' + a.compare:>8}" if a.compare else ""
    print(f"{'t[s]':>6} {'T[°C]':>7} {'SP':>6} {'err':>6} {'PWM%':>6} {'V':>5} {'I[A]':>6} {'P[W]':>6} {'R[Ω]':>5} {'Veq':>5} | {'Kp':>8} {'Ki':>8} {'Kd':>8}{cmp_hdr}  action")
    window, hist, T_prev, t0, first = [], [], None, time.time(), []
    with open(log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "T_C", "SP_C", "PWM_pct", "V", "I_A", "P_W", "R_heater_ohm", "V_equiv", "Kp", "Ki", "Kd", "sent",
                    "T_twin_classical", "IAE_real", "IAE_twin_classical"])
        f.flush()
        warned = False
        raw_seen, n_good, t_start, hinted = [], 0, time.time(), False
        last_sent = None                       # last gains the agent sent, re-sent after a Nano reset
        V_checked = False
        stuck_s, frozen = 0, False             # heater-stuck-ON watchdog
        try:
            while True:
                try:
                    raw = ser.readline().decode(errors="ignore").strip()
                except serial.SerialException as ex:          # USB dropped (e.g. heater switching noise): reconnect
                    print(f"\n  ! Serial link lost ({ex.__class__.__name__}). Reconnecting...")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    for attempt in range(60):
                        time.sleep(1)
                        try:
                            port = find_port() if a.port.lower() == "auto" else port
                            ser = serial.Serial(port, a.baud, timeout=2)
                            time.sleep(2); ser.reset_input_buffer()
                            ser.write(f"S:{a.setpoint:.2f}\n".encode())
                            if a.fixed:
                                g = classical(a.fixed)
                                ser.write(f"K:{g['Kp']:.4f},{g['Ki']:.6f},{g['Kd']:.3f}\n".encode())
                            elif last_sent and not a.shadow:
                                ser.write((last_sent + "\n").encode())   # the Nano rebooted with default gains
                            window, T_prev = [], None
                            print(f"  reconnected on {port}" + (f", re-sent {last_sent}" if last_sent and not a.shadow and not a.fixed else "")
                                  + "\n  (if this keeps happening the Nano is resetting when the heater switches: see run_on_rig/README.md)\n")
                            break
                        except (serial.SerialException, SystemExit):
                            continue
                    else:
                        print("  could not reconnect for 60 s: check the USB cable. Stopping.")
                        break
                    continue
                if raw:
                    raw_seen = (raw_seen + [raw])[-5:]
                    if a.raw:
                        print(f"  RAW: {raw}")
                d = parse(raw) if raw else None
                if not d:
                    if n_good == 0 and not hinted and time.time() - t_start > 10:
                        hinted = True
                        print("\n  ! No valid data from the Nano in 10 s.")
                        if not raw_seen:
                            print("    Nothing at all was received. Check: (1) arduino_nano_rl_pid.ino is uploaded, "
                                  "(2) the Arduino Serial Monitor is CLOSED, (3) baud 9600, (4) the right port "
                                  "(pass it explicitly, e.g. COM8, instead of auto).")
                        else:
                            print("    Lines are arriving but are not in the expected format "
                                  "'T:..,SP:..,E:..,U:..,V:..,I:..,P:..,G:kp,ki,kd'. Last lines received:")
                            for r in raw_seen:
                                print(f"      {r}")
                            print("    -> Upload run_on_rig/arduino_nano_rl_pid/arduino_nano_rl_pid.ino (an older sketch may be on the Nano).")
                        print()
                    continue
                n_good += 1
                t = time.time() - t0
                if len(first) < 5:
                    first.append(d)
                    if len(first) == 5:
                        sanity_check(first)
                        # The agent's observation uses the ambient temperature (headroom, ΔT). Using the step-test value
                        # (29.2 °C) in a 25 °C room made it over-tune at low supply voltage (limit cycle at 3.3 V).
                        T_start = float(np.median([x["T"] for x in first]))
                        if a.ambient is not None:
                            cfg.T_amb = a.ambient
                        elif T_start < d["SP"] - 3.0:
                            cfg.T_amb = T_start
                        else:
                            print(f"  ! Rig is already warm ({T_start:.1f} °C); using ambient {cfg.T_amb:.1f} °C. "
                                  "Pass --ambient <room °C> for best results.")
                        print(f"  Ambient used by the agent: {cfg.T_amb:.2f} °C")
                if not a.volts and model is not None and len(first) == 5 and not V_checked:
                    V_checked = True
                    if np.median([x["V"] for x in first]) < 0.5:
                        print("\n  ! STOP: A2 reads ~0 V, so the agent would be fed a GUESSED supply voltage (4.0 V).\n"
                              "    Fix the 25V sensor wiring, or pass the real supply, e.g.  --volts 4.0  (read it on a multimeter).")
                        break
                V = a.volts if a.volts else (d["V"] if np.isfinite(d["V"]) and d["V"] > 0.5 else 4.0)
                # live heater resistance: average current = duty * V / R  ->  R = V * duty / I  (needs some PWM and current)
                V_measured = a.volts is None and np.isfinite(d["V"]) and d["V"] > 0.5   # A2 really reads the supply
                if V_measured and d["U"] > 30 and np.isfinite(d["I"]) and d["I"] > 0.15:
                    R_heater = 0.95 * R_heater + 0.05 * float(np.clip(V * d["U"] / 100.0 / d["I"], 0.5, 200.0))
                V_eq = V * np.sqrt(cfg.R / R_heater)            # same heater power on the trained model
                if not (V_TRAIN[0] - 0.2 <= V_eq <= V_TRAIN[1] + 0.2) and not warned:
                    lo, hi = supply_range(R_heater, cfg)
                    print(f"  ! Heater power is outside what the agent was trained on (V_eq {V_eq:.1f} V). "
                          f"For this {R_heater:.1f} Ω heater keep the supply within {lo:.1f}–{hi:.1f} V, or retrain.")
                    warned = True
                # FIX 4: heater-stuck-ON watchdog. PWM is 0 but heater current keeps flowing (or T keeps rising far above
                # the setpoint): the actuator is not obeying the PID. The agent has never seen this, so freeze it.
                cur_on = np.isfinite(d["I"]) and d["I"] > 0.3
                if d["U"] < 1.0 and (cur_on or d["T"] > d["SP"] + 3.0 and T_prev is not None and d["T"] > T_prev + 0.05):
                    stuck_s += 1
                else:
                    stuck_s = 0
                if stuck_s >= 60 and not frozen:
                    frozen = True
                    print("\n  !!! HEATER NOT FOLLOWING THE PID: PWM = 0 % for 60 s but "
                          + (f"current = {d['I']:.2f} A" if cur_on else "temperature still rising")
                          + ".\n      Agent FROZEN (no more gain changes). Check the MOSFET/wiring: send M:0 and measure D6 and the heater current.\n")
                window.append(d)
                T_prev = d["T"] if T_prev is None else T_prev
                sent = ""
                if model is not None and len(window) >= RL_EVERY and frozen:
                    window, T_prev = [], d["T"]
                elif model is not None and len(window) >= RL_EVERY:
                    obs, z, base = build_obs(window, T_prev, float(np.clip(V_eq, *V_TRAIN)), cfg)  # noqa
                    act = model.predict(obs, deterministic=True)[0]
                    z = np.clip(z + STEP_Z * np.clip(act, -1, 1), -1, 1)
                    kp, ki, kd = gains_from_z(z, base)
                    sent = f"K:{kp:.3f},{ki:.5f},{kd:.2f}"
                    if not a.shadow:
                        ser.write((sent + "\n").encode())
                        last_sent = sent
                    T_prev, window = d["T"], []
                cmp_txt, T_tw = "", float("nan")
                if twin_pid is not None:
                    Vt = float(np.clip(V_eq, *V_TRAIN))
                    if twin is None:                       # twin starts where the rig is right now
                        twin = Plant(cfg, Vt, T0=d["T"])
                    twin.V = Vt
                    twin.step(twin_pid(d["SP"], twin.measure()))
                    T_tw = twin.T
                    iae_real += abs(d["SP"] - d["T"]); iae_twin += abs(d["SP"] - T_tw)
                    cmp_txt = f" {T_tw:7.2f} {iae_real:7.0f} {iae_twin:8.0f}"
                print(f"{t:6.0f} {d['T']:7.2f} {d['SP']:6.1f} {d['SP'] - d['T']:6.2f} {d['U']:6.1f} {V:5.2f} {d['I']:6.3f} {d['P']:6.2f} {R_heater:5.1f} {V_eq:5.2f} | "
                      f"{d['Kp']:8.3f} {d['Ki']:8.5f} {d['Kd']:8.2f}{cmp_txt}  {sent}{' (shadow, not sent)' if sent and a.shadow else ''}")
                w.writerow([round(t, 1), d["T"], d["SP"], d["U"], V, d["I"], d["P"], round(R_heater, 2), round(V_eq, 3),
                            d["Kp"], d["Ki"], d["Kd"], sent, round(T_tw, 3), round(iae_real, 1), round(iae_twin, 1)]); f.flush()
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
