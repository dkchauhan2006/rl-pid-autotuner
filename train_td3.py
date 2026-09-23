"""
train_td3.py

Trains the TD3 agent on the simulated ThermalPIDEnv (thermal_pid_env.py).
Run this AFTER you've fitted K_range/tau_range/theta_range from your real
step-test data (see fit_fopdt.py) and plugged them into thermal_pid_env.py.

Output: td3_pid_tuner.zip  -- this is the weights file you'll load later
for real-time deployment on the Arduino rig.

Usage:
    python train_td3.py
    tensorboard --logdir ./tb_logs/     # optional, to watch training progress
"""

import numpy as np
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback

from thermal_pid_env import ThermalPIDEnv


def make_env():
    return Monitor(ThermalPIDEnv())


if __name__ == "__main__":
    train_env = DummyVecEnv([make_env])
    eval_env = DummyVecEnv([make_env])

    n_actions = train_env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    model = TD3(
        "MlpPolicy",
        train_env,
        action_noise=action_noise,
        learning_rate=3e-4,
        buffer_size=200_000,
        learning_starts=2_000,
        batch_size=256,
        train_freq=(1, "episode"),
        gradient_steps=-1,
        policy_delay=2,          # this is TD3's "delayed actor update" trick
        target_policy_noise=0.2, # this is TD3's "target policy smoothing" trick
        verbose=1,
        tensorboard_log="./tb_logs/",
    )

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path="./best_model/",
        log_path="./eval_logs/",
        eval_freq=5_000,
        n_eval_episodes=5,
        deterministic=True,
    )

    TOTAL_TIMESTEPS = 300_000   # increase if the agent hasn't converged (watch tensorboard)
    model.learn(total_timesteps=TOTAL_TIMESTEPS, callback=eval_callback)

    model.save("td3_pid_tuner")
    print("\nTraining complete.")
    print("Weights saved to: td3_pid_tuner.zip")
    print("Best-so-far checkpoint saved to: best_model/best_model.zip")
