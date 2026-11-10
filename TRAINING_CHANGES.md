# v2 training: what was wrong with the v1 agent and what changed

## Symptoms of v1 (models/td3_best from 6 Nov)
- In the digital twin at 4 V / 40 °C it pushed its gains to the bounds: **Kp → 300 (max), Ki → 0.002 (min), Kd → 1 (min)**.
  That is an on/off controller with no integral action: ~0.2 °C steady-state offset, limit cycling.
- From bad gains it was worse than SIMC in every cold-start case of results/summary.md (e.g. 4 V: IAE 9192 vs 8002).
- Training curve: eval reward still swinging ±80–250 at 200k steps; "best" model = one lucky evaluation.

## Causes and fixes
| # | v1 problem | v2 fix | file |
|---|---|---|---|
| 1 | Gains could roam Kp 1–300, Ki 0.002–3, Kd 1–3000 (2.5 decades). The agent found a corner with Ki ≈ 0. | Gains are **anchored**: gain = SIMC(V, SP) × multiplier, multiplier ∈ [0.25, 4] (Kp, Ki), [0.05, 2] (Kd). Integral action can never be switched off; zero action = SIMC. | thermal_pid_env.py |
| 2 | Every episode started from uniformly random gains over that huge range. | 40 % of episodes start near SIMC, 60 % up to 4× wrong (the agent learns to repair). | thermal_pid_env.py |
| 3 | γ = 0.98 with a decision every 10 s → looks ~500 s ahead; the plant (τ ≈ 1000 s, θ 40–80 s) shows the effect of a gain change much later. | γ = 0.99. | train_td3.py |
| 4 | Best model chosen by mean reward over 8 *random* episodes (very noisy). | Best model chosen on a **fixed test suite** (8 scenarios × 3 starting tunings) scored as IAE / IAE(SIMC) + chatter penalty. 1.00 = SIMC accuracy. | eval_suite.py, train_td3.py |
| 5 | Sensor model = MAX6675 thermocouple only (0.25 °C steps). The rig now has an averaged NTC (measured noise 0.018 °C, 0.01 °C steps). | Sensor noise 0.01–0.12 °C and resolution 0.01–0.25 °C randomised. | plant.py |
| 6 | One seed, 200k steps. | Several seeds, 300k steps each, best seed copied to models/td3_best. | train_td3.py |
| 7 | Data read only from the local data/raw folder. | Step tests fetched from GitHub (data/raw of the repo), cached in data/github_cache, local fallback. | data_source.py, system_id.py |

The firmware and live_tuner.py were updated to the same gain scheme (the firmware's hard clamp = `SAFETY` in
thermal_pid_env.py: Kp 0.5–1000, Ki 0.0005–10, Kd 0–5000; at 2 V the plant is weak and SIMC itself needs Kp ≈ 157).

## Run it
```bash
pip install -r requirements.txt
python train_td3.py                      # GitHub data -> plant model -> 3 seeds x 300k steps -> models/td3_best
python train_td3.py --seeds 0 1          # fewer seeds (each ~1.5 h on one CPU core)
python tools/check_v2_agent.py           # 4 V / 40 °C check + results/v2_sp40_check.png
python evaluate.py                       # full comparison table (results/summary.md)
```
Then upload `run_on_rig/arduino_nano_rl_pid/arduino_nano_rl_pid.ino` again (new gain limits) and use
`run_on_rig/live_tuner.py` exactly as before (add `--volts <supply>` if A2 does not read the supply).
