# RL PID Auto-Tuner -- Training Pipeline

## Order to run things (do not skip steps)

### 1. Install dependencies
```
pip install -r requirements.txt
```

### 2. Collect real step-test data from the Arduino rig
- Flash Arduino with your PID sketch, modified to print a line every 0.5-1s:
  `T:78.4,SP:80.0,E:1.6,U:65`
- Run a step test (fixed heater output, let temperature settle):
  ```
  python log_step_test.py COM3
  ```
- This produces `step_test_data.csv`.
- **This is the only real hardware data you need** -- a handful of step
  tests (2-3 runs), not thousands of episodes.

### 3. Fit the FOPDT model from that data
```
python fit_fopdt.py
```
- Reads `step_test_data.csv`, fits K (process gain), tau (time constant),
  theta (dead time).
- Prints the `K_range`, `tau_range`, `theta_range` to paste into
  `thermal_pid_env.py`.
- Check `fopdt_fit.png` -- the fitted curve should visually match your data.
  If it doesn't, your step test was probably too short or noisy; redo it.

### 4. Update the simulator with your fitted ranges
Open `thermal_pid_env.py`, edit the `ThermalPIDEnv.__init__` defaults:
```python
K_range=(...),      # from fit_fopdt.py output
tau_range=(...),
theta_range=(...),
```
Also set `kp_bounds`, `ki_bounds`, `kd_bounds` to values you know are safe
for your actual heater (these are hard safety clamps -- the RL agent can
never command gains outside this range).

### 5. Train the TD3 agent (on the simulator, not real hardware)
```
python train_td3.py
```
- Runs 300k simulated timesteps (~20-40 min on a normal laptop CPU).
- Watch progress: `tensorboard --logdir ./tb_logs/`
- Output: `td3_pid_tuner.zip` (final weights) and `best_model/best_model.zip`
  (best checkpoint by eval reward).

### 6. (Next step, once training looks good)
Load `td3_pid_tuner.zip` with `TD3.load(...)`, connect to the Arduino over
serial (same protocol as `log_step_test.py` uses), and on each cycle:
read state from Arduino -> `model.predict(state)` -> send new Kp,Ki,Kd back.
We'll build this deployment script next.

## Why this order works with limited real data
You only need enough real data to fit **3 numbers** (K, tau, theta) -- a
handful of step tests is enough for that (regression, not RL). The RL agent
then trains on millions of *simulated* interactions around those numbers
(domain randomization), so it never needs large real datasets. The real
rig is used again only at the end, to validate/fine-tune the trained agent
in "shadow mode" before letting it control gains live.

## Notes on the safety mechanism
The proposal calls for an "inverting gradient" trick for hard safety bounds
(flips gradient sign near limits instead of clipping, so learning doesn't
saturate at the edges). Implementing that properly requires customizing
TD3's actor loss function (not exposed by stable-baselines3 out of the box).
`thermal_pid_env.py` currently uses simple hard clipping at the environment
level, which is safe and works fine for training, just less elegant. If you
want the literal inverting-gradient version for your report's novelty
claim, that's a next-step customization on top of this base -- ask when
you're ready for it.
