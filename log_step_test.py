"""
log_step_test.py

Connects to the Arduino over USB-serial and logs a step test to
step_test_data.csv in the exact format fit_fopdt.py expects.

BEFORE running this:
  - Flash your Arduino with a sketch that, every ~0.5-1s, prints a line like:
        T:78.4,SP:80.0,E:1.6,U:65
    (temperature, setpoint, error, current heater output %)
  - For the step test itself, you can either:
      (a) put the Arduino in a "manual mode" where U is a fixed value you
          set once (bypass the PID temporarily), or
      (b) just set the Setpoint high once via serial ("K:..." not needed
          here) and record the natural PID response -- less clean than an
          open-loop step but works if manual override isn't implemented yet.
    Open-loop (a) gives a much better FOPDT fit.

Usage:
    python log_step_test.py COM3          # Windows
    python log_step_test.py /dev/ttyUSB0  # Linux/Mac
"""

import sys
import time
import csv
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = 9600
DURATION_S = 300  # 5 minutes -- adjust until your temperature clearly plateaus

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # let Arduino finish its reset after serial connect

print(f"Logging from {PORT} for {DURATION_S}s... Ctrl+C to stop early.")

rows = []
start = time.time()
try:
    while time.time() - start < DURATION_S:
        line = ser.readline().decode(errors="ignore").strip()
        if not line.startswith("T:"):
            continue
        try:
            parts = dict(kv.split(":") for kv in line.split(","))
            t_now = round(time.time() - start, 2)
            rows.append([t_now, float(parts["U"]), float(parts["T"])])
            print(f"t={t_now:6.1f}s  U={parts['U']}%  T={parts['T']}C")
        except (ValueError, KeyError):
            continue  # skip malformed lines
except KeyboardInterrupt:
    print("\nStopped early by user.")

with open("step_test_data.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_s", "u_percent", "temp_C"])
    writer.writerows(rows)

print(f"\nSaved {len(rows)} rows to step_test_data.csv")
print("Next: python fit_fopdt.py")
