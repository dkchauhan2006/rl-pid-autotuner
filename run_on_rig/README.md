# Run the trained agent on the rig (Arduino Nano)

| File | What it does |
|---|---|
| `arduino_nano_rl_pid/arduino_nano_rl_pid.ino` | Flash once to the Nano. PID_v1 every 1 s on the **farthest** thermocouple, heater PWM on D9, **A0 = current**, **A1 = supply voltage**. Prints `T,SP,E,U,V,I,P,G` every second and accepts new gains `K:Kp,Ki,Kd`. |
| `live_tuner.py` | Run on the laptop. Reads the Nano in real time, prints T / PWM / V / I / P / Kp / Ki / Kd every second, runs the TD3 agent every 10 s and sends the new gains. Logs to `logs/`. |

## Wiring (Nano)
| Nano pin | Connect to |
|---|---|
| A0 | current sensor output (default ACS712-5A; set `I_SENS_V_PER_A` for 20 A / 30 A modules) |
| A1 | voltage sensor output, measured across the heater supply **before** the MOSFET (default 5:1 module; set `VOLT_RATIO`) |
| D4 / D5 / D6 | MAX6675 SO / CS / SCK of the farthest thermocouple |
| D9 | MOSFET gate or SSR input that switches the heater |
| 5V, GND | sensor modules |

## Steps
1. Arduino IDE → Library Manager → **PID** (Brett Beauregard) + **MAX6675 library** (Adafruit).
2. Open `arduino_nano_rl_pid/arduino_nano_rl_pid.ino` → Board **Arduino Nano** (clones: Processor *ATmega328P (Old Bootloader)*) → Upload.
3. Serial Monitor 9600: check that `V:` matches your supply and `I:` your multimeter (calibrate the constants if not). **Close the Serial Monitor.**
4. From the repo root:
```bash
.venv\Scripts\activate
python run_on_rig/live_tuner.py COM5 --setpoint 50 --shadow    # watch only
python run_on_rig/live_tuner.py COM5 --setpoint 50 --plot      # live RL tuning + live plot
python run_on_rig/live_tuner.py COM5 --setpoint 50 --fixed zn  # comparison: fixed ZN gains (also: simc, matlab)
```
Ctrl+C stops the agent. The Nano keeps the last gains, so the heater stays under control.

**Testing the claim:** let it settle at 4 V, switch the supply to 3 V or 5 V, and compare `logs/run_rl_*.csv` with `logs/run_zn_*.csv`.
