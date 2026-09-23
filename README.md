# RL-PID-AUTOTUNER
Reinforcement learning (TD3) agent that auto-tunes PID gains in real-time for a temperature control system, trained via a domain-randomized FOPDT simulator and deployed on an Arduino-based rig.

# RL PID Auto-Tuner

Autonomous PID gain tuning using Twin Delayed DDPG (TD3), a continuous-action
actor-critic reinforcement learning algorithm. A supervisory RL agent
observes real-time process error and adjusts Kp, Ki, Kd of an inner PID
control loop on the fly, instead of relying on fixed, manually-tuned gains.

Motivation: classical tuning methods (Ziegler-Nichols, relay auto-tune)
assume the process stays fixed over time. Real thermal/chemical processes
drift due to sensor noise, disturbances, and changing loads -- a controller
that keeps re-tuning itself is better suited to these conditions.

## System

- **Plant**: Arduino Nano + MAX6675 thermocouple + SSR-driven heater
  (temperature control testbed)
- **Inner loop**: discrete PID (position form, anti-windup) running on the
  Arduino, executing at fixed sample time
- **Outer loop**: TD3 agent (Python) observing error/state and updating
  Kp, Ki, Kd over serial, at a slower supervisory rate
- **Training**: entirely in simulation, on a First-Order-Plus-Dead-Time
  (FOPDT) digital twin fitted from real step-test data, with domain
  randomization for sim-to-real robustness

## Repo Structure

| File | Purpose |
|---|---|
| `log_step_test.py` | Logs a real step-test run from the Arduino to CSV |
| `fit_fopdt.py` | Fits FOPDT parameters (K, tau, theta) from that CSV |
| `thermal_pid_env.py` | Gymnasium environment (digital twin) used for RL training |
| `train_td3.py` | TD3 training script (stable-baselines3) |
| `requirements.txt` | Python dependencies |

See `README.md` pipeline section below for exact run order.

## Status
🚧 Work in progress -- training pipeline complete, real-time deployment
script (loading trained weights for live Arduino control) in progress.
