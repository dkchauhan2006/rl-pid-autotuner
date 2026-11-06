"""
train_td3.py  —  trains the gain-tuning agent in the domain-randomised digital twin.

    python train_td3.py                       # TD3, 200k agent steps (≈ 23 days of simulated plant time)
    python train_td3.py --algo ddpg           # DDPG baseline from the proposal
    python train_td3.py --steps 400000 --seed 1
    tensorboard --logdir tb_logs

Outputs: models/<algo>_pid_tuner.zip (final), models/<algo>_best/best_model.zip (best on eval).
"""
import argparse
import os

import numpy as np
import torch
from stable_baselines3 import DDPG, TD3
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.vec_env import DummyVecEnv

from thermal_pid_env import ThermalPIDEnv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", choices=["td3", "ddpg"], default="td3")
    ap.add_argument("--steps", type=int, default=200_000)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    torch.set_num_threads(max(1, (os.cpu_count() or 2) // 2))
    os.makedirs("models", exist_ok=True)
    train_env = DummyVecEnv([lambda: Monitor(ThermalPIDEnv(seed=a.seed))])
    eval_env = DummyVecEnv([lambda: Monitor(ThermalPIDEnv(seed=10_000 + a.seed))])

    noise = NormalActionNoise(mean=np.zeros(3), sigma=0.3 * np.ones(3))
    common = dict(policy="MlpPolicy", env=train_env, action_noise=noise, learning_rate=3e-4,
                  buffer_size=300_000, learning_starts=5_000, batch_size=256, gamma=0.98, tau=0.005,
                  train_freq=1, gradient_steps=1, policy_kwargs=dict(net_arch=[256, 256]),
                  verbose=0, seed=a.seed, tensorboard_log="tb_logs")
    if a.algo == "td3":
        # TD3's three tricks: twin critics (min of two Q), delayed actor update, target policy smoothing
        model = TD3(policy_delay=2, target_policy_noise=0.2, target_noise_clip=0.5, **common)
    else:
        model = DDPG(**common)

    cb = EvalCallback(eval_env, best_model_save_path=f"models/{a.algo}_best", log_path=f"models/{a.algo}_eval",
                      eval_freq=10_000, n_eval_episodes=8, deterministic=True, verbose=1)
    model.learn(total_timesteps=a.steps, callback=cb, progress_bar=False, tb_log_name=a.algo)
    model.save(f"models/{a.algo}_pid_tuner")
    print(f"saved models/{a.algo}_pid_tuner.zip and models/{a.algo}_best/best_model.zip")


if __name__ == "__main__":
    main()
