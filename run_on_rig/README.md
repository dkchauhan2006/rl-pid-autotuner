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

## Assumed specs (typical values; replace them if you have the datasheets)
| Part | What we assumed | Where it matters |
|---|---|---|
| 24 V 40 W cartridge heater | R = 24²/40 ≈ **14.4 Ω**, ≈ 1.67 A at 24 V, max 24 V | `live_tuner.py --heater-ohms` (default 14.4); it is then **measured live** from A2/A1 |
| B3950 100K NTC | R25 = 100 kΩ, Beta = 3950; **NTC to 5 V, 100 kΩ to GND** (A0 = 2.5 V at 25 °C → 4.4 V at 80 °C, always 2–5 V) | `R_FIXED = 100000`, `NTC_TO_GND = false` |
| ACS712 30A | 66 mV/A, 2.5 V at 0 A (auto-zeroed at start-up) | `I_SENS_V_PER_A` |
| 25V voltage module | 30 k / 7.5 k divider → ratio 5, max 25 V | `VOLT_RATIO` |
| STP55NF06 | Vgs(th) 2–4 V, not logic-level; fine for ≤ 2 A at 5 V gate drive (add a small heat sink) | D5 gate drive |
| Nano ADC | 10-bit, reference = USB 5 V | `ADC_REF` (measure your 5 V pin) |

### ⚠️ Supply voltage: 2–5 V
Our step tests were done at **2, 3, 4 and 5 V** and measured the heater at R = V/I ≈ **3.7 Ω**; the agent was trained on
that range (1.8–5.5 V). Run the heater supply at **2–5 V** (as our professor asked). `live_tuner.py` assumes 3.7 Ω until it
has measured the heater itself from A2/A1, and warns if the heater power leaves the trained range.

### If V or I stay at 0 while the heater heats up
- **A2 (25V voltage sensor):** its blue screw terminal goes **across the heater supply**: `VCC` to supply (+), `GND` to supply (−).
  Its 3-pin side: `S` → A2, `−` → Nano GND (`+` is not used).
- **A1 (ACS712-30A):** the heater current must flow **through its two screw terminals** (in series with the heater).
  `VCC` → 5V, `GND` → GND, `OUT` → A1.
- Nano GND, supply (−) and MOSFET source must be connected together (common ground).

## Pins
| Nano | Connect to |
|---|---|
| **A0** | NTC divider: 5V → NTC → **A0** → 100 kΩ → GND |
| **A1** | ACS712-30A `OUT` (VCC 5V, GND); heater current through its screw terminals |
| **A2** | 25V voltage sensor `S`; its input terminals across the heater **supply** (before the MOSFET), `-` to GND |
| **D5** | 220 Ω → STP55NF06 gate, plus 10 kΩ gate → GND |
| GND | common ground: Nano GND = heater supply GND = MOSFET source |

MOSFET: drain → heater (−), heater (+) → supply (+), source → supply GND.
The STP55NF06 is not logic-level: at 5 V gate drive it is fine for ~1–2 A. For more current use a logic-level MOSFET (IRLZ44N) or a gate driver.

## Files
| File | Runs on | What it does |
|---|---|---|
| `arduino_nano_rl_pid/arduino_nano_rl_pid.ino` | **Nano** (upload once) | PID_v1 every 1 s on the NTC temperature, heater PWM on D5, reads A1 current and A2 voltage. Prints `T,SP,E,U,V,I,P,G` every second, accepts `K:Kp,Ki,Kd`. All assumed specs are listed at the top of the file |
| `live_tuner.py` | **Laptop** | Finds the CH340 Nano, checks the sensors, measures the heater resistance, prints T / PWM / V / I / P / R / V_eq / Kp / Ki / Kd every second, runs the TD3 agent every 10 s and sends the new gains. Logs to `logs/` |

## Steps
1. Windows: install the **CH340 driver** if the Nano doesn't show up as a COM port.
2. Arduino IDE → Library Manager → **PID** by Brett Beauregard (the only library).
3. Open `arduino_nano_rl_pid/arduino_nano_rl_pid.ino` → Board **Arduino Nano**, Processor **ATmega328P** (our board; use *Old Bootloader* only if the upload fails with 'not in sync') → Upload.
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

## Compare with the best classical tuning (SIMC)
**Live, side by side** (RL on the real rig, SIMC on the digital twin with the same setpoint and voltage):
```bash
python run_on_rig/live_tuner.py COM8 --setpoint 50 --compare simc      # also: zn, matlab
```
Extra columns `T_simc`, `IAE_rl`, `IAE_simc` show both controllers every second.

**Real A/B test on the rig** (the fair comparison): start both runs from room temperature with the same setpoint:
```bash
python run_on_rig/live_tuner.py COM8 --setpoint 50                 # RL run, then Ctrl+C and let it cool down
python run_on_rig/live_tuner.py COM8 --setpoint 50 --fixed simc    # best classical gains
python run_on_rig/compare_logs.py                                  # table + logs/comparison.png
```

## Uploading from the command line (what we used)
```powershell
C:\Users\deepa\tools\arduino-cli\arduino-cli.exe compile --upload -p COM8 --fqbn arduino:avr:nano:cpu=atmega328 run_on_rig\arduino_nano_rl_pid
```
Our Nano uses the **new** bootloader (`cpu=atmega328`, in the IDE: Processor *ATmega328P*). The firmware also prints the raw
A0 reading as `A:`; if `T:` looks wrong, `live_tuner.py` shows what that reading means with other divider resistors.

**Testing the claim:** settle at 4 V, switch the supply to 3 V or 5 V, and compare `logs/run_rl_*.csv` with `logs/run_zn_*.csv`.
