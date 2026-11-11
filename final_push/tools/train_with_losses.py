"""
tools/train_with_losses.py  —  re-run the v2 training of one seed with identical settings (train_td3.py) and also
save the TD3 losses, which train_td3.py does not keep unless tensorboard is installed.

    python tools/train_with_losses.py [--seed 0] [--steps 300000]

Outputs in results/training_losses/seed<k>/:
    progress.csv     per episode: total steps, training episode return, critic loss, actor loss
    suite_log.csv    test-suite score (IAE / IAE_SIMC) every 20k steps — identical to models/td3_seed<k>/suite_log.csv
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)
import numpy as np                                                   # noqa: E402
import torch                                                         # noqa: E402
from stable_baselines3 import TD3                                    # noqa: E402
from stable_baselines3.common.logger import configure                # noqa: E402
from stable_baselines3.common.monitor import Monitor                 # noqa: E402
from stable_baselines3.common.noise import NormalActionNoise         # noqa: E402
from stable_baselines3.common.vec_env import DummyVecEnv             # noqa: E402
from thermal_pid_env import ThermalPIDEnv                            # noqa: E402
from train_td3 import SuiteCallback                                  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--steps", type=int, default=300_000)
a = ap.parse_args()
torch.set_num_threads(1)
out = os.path.join("results", "training_losses", f"seed{a.seed}")
env = DummyVecEnv([lambda: Monitor(ThermalPIDEnv(seed=a.seed))])
m = TD3("MlpPolicy", env, action_noise=NormalActionNoise(np.zeros(3), 0.3 * np.ones(3)), learning_rate=3e-4,
        buffer_size=a.steps, learning_starts=5_000, batch_size=256, gamma=0.99, tau=0.005, train_freq=1, gradient_steps=1,
        policy_kwargs=dict(net_arch=[256, 256]), verbose=0, seed=a.seed, policy_delay=2, target_policy_noise=0.2,
        target_noise_clip=0.5)
m.set_logger(configure(out, ["csv"]))
cb = SuiteCallback(out, 20_000)
m.learn(total_timesteps=a.steps, callback=cb.cb, log_interval=1)
print("wrote", out)
