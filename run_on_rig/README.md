# 🔥 RL PID Auto-Tuner: a TD3 agent that re-tunes a heater PID live

**Autonomous PID parameter tuning via actor–critic reinforcement learning on a low-cost Arduino temperature rig.**
Team: Deepak Kumar Chauhan · Azaad Katiyar · Divy Kumar Pandey · Proposal: [`docs/PID_AUTOTUNER_FINAL.pdf`](docs/PID_AUTOTUNER_FINAL.pdf)

🎤 **Presentation:** open [`results/presentation.html`](results/presentation.html) in a browser (← → keys, Ctrl+P for PDF) ·
🔌 **Run on the rig (Arduino Nano):** [`run_on_rig/`](run_on_rig/)

The Arduino runs a fast PID loop (the **reflex**). A TD3 agent on the laptop (the **brain**) watches the loop and nudges
`Kp, Ki, Kd` every 10 s. It was trained in a digital twin fitted to **our own step-test data**, and it keeps working when the
**supply (inlet) voltage changes** or the plant drifts. Classical tunings (Ziegler–Nichols, Cohen–Coon, SIMC, AMIGO, IMC, MATLAB
`pidtune`) are computed once for one operating point and then stay fixed.

---

## 📊 Results (digital twin of our rig, farthest thermocouple)

**40 random plants** (process gain, nonlinearity, time constant, dead time, heater resistance, ambient, sensor noise all
randomised) with **random supply-voltage changes (1.8–5.5 V) and setpoint steps**. Same plant, scenario and noise for every controller:

| Controller | Mean IAE (°C·s) ↓ | Median IAE ↓ | Not settled at the end ↓ | PWM chatter (%/s) ↓ |
|---|---|---|---|---|
| Ziegler–Nichols (tuned at 4 V) | 8581 | 7090 | 12 / 40 | 42.9 |
| Cohen–Coon | 7735 | 6647 | 14 / 40 | 41.2 |
| SIMC (≈ MATLAB-style IMC tuning) | 6996 | 6023 | 10 / 40 | 39.2 |
| AMIGO | 8143 | 7013 | 12 / 40 | 41.6 |
| IMC (λ tuning) | 8562 | 6940 | 17 / 40 | 36.4 |
| **MATLAB `pidtune`** (PI, tuned at 4 V) | 9849 | 8765 | 12 / 40 | 0.9 |
| **TD3 agent, starting from deliberately bad gains** | **5945** | **4827** | **2 / 40** | **10.8** |
| **TD3 agent, starting from ZN gains** | **5706** | **4422** | 5 / 40 | 11.4 |

**In one line:** the agent is **15–18 % more accurate than the best classical tuning** and **42 % more accurate than MATLAB
`pidtune`** on average. It leaves far fewer runs unsettled and switches the heater **~4× less violently than ZN / Cohen–Coon / SIMC**.
It needs no step test or re-tuning when the voltage changes. MATLAB's PI is the smoothest (0.9 %/s) because it gives up speed:
2–3× slower settling and up to 15 % overshoot.

### What the agent learned: a supply jump from 3 V to 5 V (heater power × 2.8)
![supply rise](results/supply_rise_3to5V.png)
At the red line the supply jumps. The agent **lowers Kp from ~150 to ~12 and raises Ki** (more voltage needs less gain),
settles back on the setpoint and holds the PWM smoothly at ~27 %. The fixed-gain controllers wander above the setpoint for
a long time, and ZN switches the heater between 0 and 100 % every second for the whole run.

| Scenario (all 90 min) | Best classical IAE | MATLAB `pidtune` IAE | TD3 IAE (from ZN gains) | Chatter: classical → TD3 |
|---|---|---|---|---|
| Cold start, 2 V | 2938 (ZN re-tuned for 2 V) | 7244 | **2659** | 45 → 33 %/s |
| Cold start, 3 V | 5437 (SIMC) | 9238 | **5119** | 43 → 16 %/s |
| Cold start, 4 V (tuning point) | 8002 (Cohen–Coon) | 10420 | **7889** | 43 → 10 %/s |
| Cold start, 5 V | **10937** (SIMC) | 14902 | 11292 (+3 %) | 43 → 21 %/s |
| Supply rise 3 → 5 V | 1090 (Cohen–Coon) | 5391 | **537** | 47 → 7 %/s |
| Supply drop 5 → 3 V | **593** (ZN re-tuned) | 5494 | 1411 | 49 → 18 %/s |
| Setpoint step at 4 V | 5177 (SIMC) | 5995 | **4373** | 42 → 13 %/s |

### MATLAB `pidtune` on our identified model
`matlab/pidtune_compare.m` builds `G(s) = K·e^(−θs)/(τs + 1)` for each supply voltage from `models/plant_params.json`
(farthest sensor, input = PWM %) and runs `pidtune(G, 'PID')`. Output: [`results/matlab_pidtune.csv`](results/matlab_pidtune.csv).

| Supply | MATLAB Kp | MATLAB Ki | MATLAB Kd | 4 V gains on this plant: overshoot | settling |
|---|---|---|---|---|---|
| 2 V | 16.379 | 0.02824 | 0 | 3.2 % | 6250 s |
| 3 V | 7.752 | 0.01407 | 0 | 6.7 % | 4129 s |
| **4 V (used as the baseline)** | **4.700** | **0.00906** | **0** | 7.3 % | 2957 s |
| 5 V | 3.261 | 0.00671 | 0 | 6.7 % | 2253 s |

Two things stand out:
1. **MATLAB picks a PI controller (Kd = 0) at every voltage.** It is very gentle, which is why its PWM chatter is the lowest (0.9 %/s),
   but it settles 2–3× slower than the agent, overshoots up to 15 % in our saturating plant, and its IAE is 42 % higher than the agent's.
2. **MATLAB needs different gains at every voltage**: Kp falls 5× from 2 V to 5 V. With fixed 4 V gains, a real rig would need a new
   step test and a new `pidtune` run every time the supply changes. That is exactly the re-tuning the agent does on its own, live.

(`pidtune`'s settling times above come from a linear model without actuator limits. Our evaluation uses the full nonlinear twin with
0–100 % PWM saturation, sensor noise and the PID_v1 algorithm, so the numbers differ.)

Full tables: [`results/summary.md`](results/summary.md) · plots: [`results/`](results/) · training curve: [`results/training_curve.png`](results/training_curve.png)

### Honest limitations
- **All results are in the digital twin.** It reproduces our step tests to 0.4 °C RMS, but real-rig validation is the next step (guide below).
- **A supply drop (5 → 3 V) is the agent's weak case:** ZN recovers with about half the IAE, although the peak deviation is similar (0.78 vs 0.63 °C).
- **From deliberately bad gains the first heat-up is slower** (the agent is still tuning). Starting from ZN gains removes most of that cost.
- At its tuning point, ZN's *accuracy* is hard to beat on this plant (θ/τ ≈ 0.04 makes it an easy, lag-dominant loop). The agent's
  advantage is **staying good when conditions change** and **much gentler actuation**, not a big accuracy win at one fixed point.
- MATLAB `pidtune` was run on the same identified model (`matlab/pidtune_compare.m`). It chose a PI controller (Kd = 0), which is
  why its PWM is the smoothest of all; the agent trades a little smoothness for much better tracking.

---

## 🧪 Step 1: what our step tests told us (system identification)

Four open-loop runs (`data/raw/samplerun_*V.txt`): heater on at a fixed supply voltage, 5 thermocouples logged every second by CoolTerm.
Supply current and switch-on times are from the lab notebook ([`docs/lab_notebook_readings.jpg`](docs/lab_notebook_readings.jpg)):

| Run | Current | Heater power | Rise of the farthest sensor | Process gain | Time constant | Dead time |
|---|---|---|---|---|---|---|
| 2 V | 0.51 A | 1.02 W | 8.1 °C | 7.92 °C/W | 944 s | (no timestamps in log) |
| 3 V | 0.81 A | 2.43 W | 18.2 °C | 7.50 °C/W | 816 s | (no timestamps in log) |
| 4 V | 1.08 A | 4.32 W | 31.6 °C | 7.30 °C/W | 915 s | 40 s |
| 5 V | 1.38 A | 6.90 W | 47.0 °C | 6.80 °C/W | 838 s | 65 s |

![system id](results/system_id_fit.png)

What we learned:
1. **Heater resistance ≈ 3.7 Ω, so power ∝ V².** Going from 2 V to 5 V multiplies the loop gain by ~6.8. A PID tuned at one voltage is wrong at another.
2. **Heat loss grows with temperature**, so the gain drops as the rig gets hotter. One nonlinear model fits all four runs (RMSE 0.40 °C):
   `τ·dT/dt = K₀·P(t−θ) − ΔT·(1 + β·ΔT)`, with K₀ = 8.51 °C/W, β = 0.0054 /°C, τ = 1141 s, θ = 40 s, and `P = (PWM%/100)·V²/R`.
3. **The rig is slow** (τ_eff ≈ 15 min), so episodes are 90 min and the agent acts every 10 s.

The log parser re-joins samples that CoolTerm split over two lines and removes thermocouple glitches (single readings that jump
by more than 3 °C in one second).

---

## 🧠 How the agent works

| | |
|---|---|
| Fast loop (Arduino, 1 s) | `PID_v1`: integral stored in output units (bumpless gain changes, anti-windup), derivative on measurement, 0–100 % PWM |
| Slow loop (laptop, 10 s) | TD3 policy → `[ΔKp, ΔKi, ΔKd]`, applied **multiplicatively in log space** and clipped to hard bounds (Kp 1–300, Ki 0.002–3, Kd 1–3000), clipped again on the Arduino |
| Observation (10 values) | error, temperature slope, mean PWM, PWM chatter, current gains, **measured supply voltage**, setpoint headroom, temperature above ambient |
| Reward | `−|e|/2 − 2·overshoot/2 − |Δu|/100 − 0.02·|action|`: track tightly, don't overshoot, don't hammer the heater (the proposal's λ·Δu² idea) |
| Training | 200 k decisions (≈ 23 days of simulated plant time) in a **domain-randomised** twin: every episode draws a new plant and a voltage/setpoint disturbance. TD3 = twin critics + delayed actor + target smoothing |

---

## 📁 Repository layout

```
├── data/raw/                 step tests from the rig (2 V, 3 V, 4 V, 5 V; 5 thermocouples, 1 Hz)
├── docs/                     proposal PDF, lab-notebook photo (currents + switch-on times)
├── models/
│   ├── plant_params.json     identified plant model + randomisation ranges   (system_id.py)
│   ├── td3_best/best_model.zip   ⭐ trained agent (best on evaluation)        (train_td3.py)
│   ├── td3_pid_tuner.zip     agent at the end of training
│   └── td3_eval/             evaluation curve during training
├── results/                  presentation.html (slide deck), summary.md, metrics.json, plots, training curve
├── arduino/pid_rig/          Arduino sketch: PID loop + serial protocol + safety cutoff
├── matlab/                   pidtune_compare.m (MATLAB built-in tuning on the same model)
├── tools/                    CoolTerm, FTDI driver, CoolTerm settings used for logging
├── run_on_rig/               ⭐ Arduino Nano firmware (A0 NTC, A1 current, A2 voltage) + live_tuner.py (real-time tuning)
├── system_id.py              step 1: rig data → plant model
├── plant.py                  digital twin + Arduino-equivalent PID + metrics
├── tuning_rules.py           ZN, Cohen–Coon, SIMC, AMIGO, IMC
├── thermal_pid_env.py        Gymnasium environment (two-speed loop, voltage changes, randomisation)
├── train_td3.py              step 2: train TD3 (or --algo ddpg)
├── evaluate.py               step 3: TD3 vs classical, tables + plots
├── deploy_rl.py              step 4: run the agent on the real rig (shadow or live)
└── log_step_test.py          record a new open-loop step test from the rig
```

---

## 💻 Reproduce everything on a PC

```bash
git clone https://github.com/dkchauhan2006/rl-pid-autotuner.git
cd rl-pid-autotuner
python -m venv .venv
.venv\Scripts\activate                     # Linux/Mac: source .venv/bin/activate
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install -r requirements.txt

python system_id.py                        # rig data → models/plant_params.json, results/system_id_fit.png
python train_td3.py                        # ~1 hour on a laptop CPU (incl. evaluations) → models/td3_best/best_model.zip
python evaluate.py --random 40             # → results/summary.md + plots
python train_td3.py --algo ddpg            # optional: DDPG baseline from the proposal
tensorboard --logdir tb_logs               # optional: watch training
```

The trained agent is already in `models/`, so you can skip training and go straight to `evaluate.py` or the rig.

---

## 🔌 Run it on the real rig (PC ↔ Arduino)

### ⭐ Our rig: Arduino Nano R3 (CH340) → use [`run_on_rig/`](run_on_rig/)
| Part | Exact model | Nano pin |
|---|---|---|
| Temperature | B3950 100K NTC thermistor (farthest point), 100 kΩ divider | **A0** |
| Current | ACS712 30A module | **A1** |
| Supply voltage | Voltage Detection Sensor Module 25V | **A2** |
| Heater switch | STP55NF06 MOSFET (100 Ω gate resistor, 10 kΩ pull-down) | **D6** |
| Heater | 24 V 40 W ceramic cartridge | MOSFET drain |

1. Flash [`run_on_rig/arduino_nano_rl_pid/arduino_nano_rl_pid.ino`](run_on_rig/arduino_nano_rl_pid/arduino_nano_rl_pid.ino)
   (library: *PID* by Brett Beauregard; board *Arduino Nano*, processor *ATmega328P (Old Bootloader)*; CH340 driver on Windows).
2. Close the Serial Monitor, then from the repo root:
```bash
python run_on_rig/live_tuner.py auto --setpoint 50 --shadow    # watch only: prints the gains it would send
python run_on_rig/live_tuner.py auto --setpoint 50 --plot      # live RL tuning, live table + plot
python run_on_rig/live_tuner.py auto --setpoint 50 --fixed zn  # comparison run (also: simc, matlab)
```
Every second it prints `t · T · SP · err · PWM% · V · I · P | Kp · Ki · Kd · action`. Every 10 s the TD3 agent sends new gains.
Logs: `run_on_rig/logs/*.csv`. Wiring details and calibration: [`run_on_rig/README.md`](run_on_rig/README.md).

### Generic Arduino (Uno etc.) without current sensing: `arduino/pid_rig` + `deploy_rl.py`

### 1. Hardware
| Part | Connection (edit the pin numbers at the top of `pid_rig.ino` to match your wiring) |
|---|---|
| MAX6675 of the **farthest** thermocouple | SO → D4, CS → D5, SCK → D6, VCC → 5 V, GND → GND |
| Heater driver (SSR / logic-level MOSFET) | control input → D9 (time-proportioned PWM, 1 s window) |
| Supply-voltage sense (recommended) | heater supply → 100 kΩ → A0 → 10 kΩ → GND (ratio 11). Lets the agent "see" the inlet voltage |
| Arduino | USB to the laptop (FTDI boards: install `tools/FTDI_driver.zip` if Windows doesn't see it) |

> ⚠️ Keep a person near the rig on the first runs. The sketch forces the heater OFF above 110 °C (`T_CUTOFF`), and the gains are
> clamped on the Arduino too, but a hardware fuse or thermal cutoff on the heater is still a good idea.

### 2. Flash the Arduino
1. Arduino IDE → Library Manager → install **"PID" by Brett Beauregard** and **"MAX6675 library" by Adafruit**.
2. Open `arduino/pid_rig/pid_rig.ino`, select your board and COM port, Upload.
3. Serial Monitor at 9600 baud should print one line per second:
   `T:29.25,SP:45.00,E:15.75,U:100.0,V:4.02,G:20.000,0.05000,100.00`
4. **Close the Serial Monitor and CoolTerm.** Only one program can use the COM port.

### 3. Find the COM port
Windows: Device Manager → *Ports (COM & LPT)* (e.g. `COM3`). Linux: `/dev/ttyUSB0` or `/dev/ttyACM0`. Mac: `/dev/tty.usbserial-*`.

### 4. Shadow mode first (the agent only watches)
```bash
.venv\Scripts\activate
python deploy_rl.py COM3 --setpoint 50 --shadow
```
The Arduino runs its default PID. Every 10 s the script prints the gains the agent **would** send, e.g.
`T= 41.25 SP= 50.0 U=100.0% V=4.01  ->  K:23.104,0.05613,104.70  (shadow)`.
Let it run through one heat-up and check that the suggestions look sensible (no huge jumps, gains settle once the temperature does).

### 5. Live tuning
```bash
python deploy_rl.py COM3 --setpoint 50
```
Now the gains are sent (`K:…`) and applied with `SetTunings` without stopping the loop. Everything is logged to `deploy_log.csv`
(time, T, SP, PWM, V, gains). Ctrl+C stops the agent; **the Arduino keeps controlling with the last gains** (safe by design).
- No voltage-sense divider? Tell the agent the supply voltage: `--volts 4.0`
- Change the setpoint while running: type `S:55` into any serial terminal, or restart the script with a new `--setpoint`.

### 6. Test the claim on the rig
1. Run at 4 V until the temperature settles, then **switch the supply to 3 V or 5 V** and watch the agent re-tune (`deploy_log.csv`).
2. Repeat with fixed ZN gains for comparison: send `K:<Kp>,<Ki>,<Kd>` from `results/summary.md` and don't run `deploy_rl.py`.
3. Compare IAE and PWM chatter from the two logs.

### 7. If the rig changes (new heater, enclosure, sensor position)
```bash
python log_step_test.py COM3 --u 100 --minutes 90     # open-loop step test → step_test_data.csv
```
Add the new runs to `data/raw/`, update `RUNS` in `system_id.py`, then `python system_id.py && python train_td3.py`.
The model only needs these few step tests. The agent itself learns entirely in the simulator.

---

## Next steps
- Validate on the real rig with voltage switching (section 6) and publish the logs next to the simulation results.
- DDPG baseline run (`train_td3.py --algo ddpg`) and the proposal's inverting-gradient safety bound (currently hard clipping).
