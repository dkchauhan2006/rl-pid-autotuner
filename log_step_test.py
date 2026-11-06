"""
log_step_test.py  —  records a new open-loop step test from the rig (to refresh the plant model).

Uses arduino/pid_rig: sends M:<u> (manual PWM) so the heater runs open loop at a fixed output,
then logs every line to step_test_data.csv.

    python log_step_test.py COM3 --u 100 --minutes 90

Note: the plant time constant is ~15–20 min, so let it run 60–90 min until the temperature plateaus.
Afterwards the Arduino is switched back to automatic (M:-1).
"""
import argparse
import csv
import time

import serial


def parse(line):
    try:
        d = dict(kv.split(":", 1) for kv in line.strip().split(",") if ":" in kv)  # G:Kp,Ki,Kd has extra commas
        return float(d["T"]), float(d["U"]), float(d.get("V", "nan"))
    except (KeyError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--u", type=float, default=100.0, help="fixed PWM %% during the test")
    ap.add_argument("--minutes", type=float, default=90.0)
    ap.add_argument("--baud", type=int, default=9600)
    a = ap.parse_args()

    ser = serial.Serial(a.port, a.baud, timeout=2)
    time.sleep(2)
    ser.write(f"M:{a.u}\n".encode())
    print(f"Open-loop step: PWM {a.u}% for {a.minutes} min. Ctrl+C to stop early.")
    rows, t0 = [], time.time()
    try:
        while time.time() - t0 < a.minutes * 60:
            p = parse(ser.readline().decode(errors="ignore"))
            if p:
                T, U, V = p
                rows.append([round(time.time() - t0, 1), U, T, V])
                print(f"t={rows[-1][0]:7.0f}s  U={U:5.1f}%  T={T:6.2f}°C  V={V:.2f}")
    except KeyboardInterrupt:
        print("stopped early")
    finally:
        ser.write(b"M:-1\n")
    with open("step_test_data.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["time_s", "u_percent", "temp_C", "supply_V"]); w.writerows(rows)
    print(f"saved {len(rows)} rows to step_test_data.csv")


if __name__ == "__main__":
    main()
