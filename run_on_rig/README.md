# Run the trained agent on our rig (Arduino Nano R3 CH340)

## Parts
| Part | Exact model |
|---|---|
| Board | Nano R3, CH340 USB chip (Arduino-compatible) |
| Temperature | B3950 100K NTC thermistor, glass head, 1 m (at the **farthest** point from the heater) |
| Current | ACS712 30A hall-effect module (66 mV/A) |
| Voltage | Voltage Detection Sensor Module 25V (5:1 divider) |
| Switch | STP55NF06 N-channel MOSFET, 60 V 50 A, TO-220 |
| Heater | 24 V 40 W ceramic cartridge heater |

## Pins
| Nano | Connect to |
|---|---|
| **A0** | NTC divider: 5V → 100 kΩ → **A0** → NTC → GND |
| **A1** | ACS712-30A `OUT` (VCC 5V, GND); heater current through its screw terminals |
| **A2** | 25V voltage sensor `S`; its input terminals across the heater **supply** (before the MOSFET), `-` to GND |
| **D9** | 220 Ω → STP55NF06 gate, plus 10 kΩ gate → GND |
| GND | common ground: Nano GND = heater supply GND = MOSFET source |

MOSFET: drain → heater (−), heater (+) → supply (+), source → supply GND.
The STP55NF06 is not logic-level: at 5 V gate drive it is fine for ~1–2 A. For more current use a logic-level MOSFET (IRLZ44N) or a gate driver.

## Files
| File | Runs on | What it does |
|---|---|---|
| `arduino_nano_rl_pid/arduino_nano_rl_pid.ino` | **Nano** (upload once) | PID_v1 every 1 s on the NTC temperature, heater PWM on D9, reads A1 current and A2 voltage. Prints `T,SP,E,U,V,I,P,G` every second, accepts `K:Kp,Ki,Kd` |
| `live_tuner.py` | **Laptop** | Finds the CH340 Nano, checks the sensors, prints T / PWM / V / I / P / Kp / Ki / Kd every second, runs the TD3 agent every 10 s and sends the new gains. Logs to `logs/` |

## Steps
1. Windows: install the **CH340 driver** if the Nano doesn't show up as a COM port.
2. Arduino IDE → Library Manager → **PID** by Brett Beauregard (the only library).
3. Open `arduino_nano_rl_pid/arduino_nano_rl_pid.ino` → Board **Arduino Nano**, Processor **ATmega328P (Old Bootloader)** → Upload.
4. Serial Monitor 9600: `T:` ≈ room temperature, `V:` = your supply (multimeter), `I:` = your ammeter. If not, edit the calibration
   constants at the top of the .ino (`R_FIXED`, `NTC_TO_GND`, `VOLT_RATIO`, `I_SENS_V_PER_A`, `ADC_REF`). **Close the Serial Monitor.**
5. From the repo root:
```bash
.venv\Scripts\activate
python run_on_rig/live_tuner.py auto --setpoint 50 --shadow    # watch only ("auto" finds the CH340 Nano)
python run_on_rig/live_tuner.py auto --setpoint 50 --plot      # live RL tuning + live plot
python run_on_rig/live_tuner.py auto --setpoint 50 --fixed zn  # comparison: fixed ZN gains (also: simc, matlab)
```
Ctrl+C stops the agent. The Nano keeps the last gains, so the heater stays under control.

**Testing the claim:** settle at 4 V, switch the supply to 3 V or 5 V, and compare `logs/run_rl_*.csv` with `logs/run_zn_*.csv`.
