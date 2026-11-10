"""
train_td3.py  —  full pipeline: GitHub step-test data -> plant model -> train the gain-tuning agent.

    python train_td3.py                          # fetch data/raw from GitHub, refit the plant, train seeds 0 1 2
    python train_td3.py --seeds 0 --steps 300000 # one seed
    python train_td3.py --source local           # use data/raw/ already on disk
    python train_td3.py --skip-sysid             # keep models/plant_params.json as it is
    python train_td3.py --pick-best              # only compare finished seeds and copy the best one
    tensorboard --logdir tb_logs

Outputs
  models/td3_seed<k>/best_model.zip   best checkpoint of seed k on the fixed test suite (eval_suite.py)
  models/td3_seed<k>/suite_log.csv    test-suite score every --eval-every steps
  models/td3_best/best_model.zip      best seed overall  -> used by live_tuner.py and evaluate.py

What went wrong in v1 and what is different now (details in thermal_pid_env.py):
  - gains could roam Kp 1–300, Ki 0.002–3 -> agent pinned Kp at 300 and Ki at 0.002 (bang-bang, no integral).
    Now gains are SIMC(V, SP) × [0.25, 4]: integral can never be switched off, zero action = SIMC.
  - episodes started from absurd random gains -> now 40 % near SIMC, 60 % up to 4x wrong.
  - gamma 0.98 (~500 s look-ahead, the plant needs ~1000 s) -> 0.99.
  - "best" model chosen on mean reward of 8 random episodes (±100–250 noise) -> fixed test suite vs SIMC.
  - sensor model was the old thermocouple only (0.25 °C) -> randomised, includes the NTC now on the rig.
  - one seed, 200k steps -> several seeds, 300k+ steps, the best seed is kept.
"""
import argparse
import csv
import glob
import json
import os
import shutil

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
try:                                   # tensorboard logging is optional
    import tensorboard  # noqa: F401
    TB = "tb_logs"
except ImportError:
    TB = None


class SuiteCallback:
    """Every eval_every steps: score the deterministic policy on the fixed suite, keep the best checkpoint."""

    def __init__(self, out_dir, eval_every):
        from stable_baselines3.common.callbacks import BaseCallback

        outer = self

        class _CB(BaseCallback):
            def _on_step(self):
                if self.n_calls % eval_every == 0:
                    outer.evaluate(self.model, self.num_timesteps)
                return True

        self.cb, self.out_dir, self.best = _CB(), out_dir, np.inf
        os.makedirs(out_dir, exist_ok=True)
        self.log_path = os.path.join(out_dir, "suite_log.csv")
        with open(self.log_path, "w", newline="") as f:
            csv.writer(f).writerow(["steps", "score_vs_SIMC", "from_SIMC", "from_low", "from_high", "worst"])

    def evaluate(self, model, steps):
        from eval_suite import score
        s, rows = score(lambda o: model.predict(o, deterministic=True)[0])
        by = {st: float(np.mean([r["ratio"] for r in rows if r["start"] == st])) for st in ("SIMC", "low", "high")}
        worst = float(max(r["ratio"] for r in rows))
        tag = ""
        if s < self.best:
            self.best = s
            model.save(os.path.join(self.out_dir, "best_model"))
            json.dump({"steps": int(steps), "score_vs_SIMC": s, **{f"from_{k}": v for k, v in by.items()},
                       "worst": worst, "rows": rows}, open(os.path.join(self.out_dir, "best_suite.json"), "w"), indent=1)
            tag = "  <- best, saved"
        with open(self.log_path, "a", newline="") as f:
            csv.writer(f).writerow([steps, round(s, 4), round(by["SIMC"], 4), round(by["low"], 4), round(by["high"], 4), round(worst, 4)])
        print(f"[{os.path.basename(self.out_dir)}] {steps:>7} steps  IAE/SIMC = {s:.3f}  "
              f"(from SIMC {by['SIMC']:.3f}, from 4x-low {by['low']:.3f}, from 4x-high {by['high']:.3f}, worst {worst:.2f}){tag}",
              flush=True)


def train_seed(seed, a):
    import torch
    from stable_baselines3 import DDPG, TD3
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.noise import NormalActionNoise
    from stable_baselines3.common.vec_env import DummyVecEnv
    from thermal_pid_env import ThermalPIDEnv

    torch.set_num_threads(a.threads)
    out = os.path.join("models", f"{a.algo}_seed{seed}")
    train_env = DummyVecEnv([lambda: Monitor(ThermalPIDEnv(seed=seed))])
    noise = NormalActionNoise(mean=np.zeros(3), sigma=0.3 * np.ones(3))
    common = dict(policy="MlpPolicy", env=train_env, action_noise=noise, learning_rate=3e-4,
                  buffer_size=a.steps, learning_starts=5_000, batch_size=256, gamma=a.gamma, tau=0.005,
                  train_freq=1, gradient_steps=1, policy_kwargs=dict(net_arch=[256, 256]),
                  verbose=0, seed=seed, tensorboard_log=TB)
    if a.algo == "td3":   # twin critics (min of two Q), delayed actor update, target policy smoothing
        model = TD3(policy_delay=2, target_policy_noise=0.2, target_noise_clip=0.5, **common)
    else:
        model = DDPG(**common)
    cb = SuiteCallback(out, a.eval_every)
    model.learn(total_timesteps=a.steps, callback=cb.cb, progress_bar=False, tb_log_name=f"{a.algo}_seed{seed}")
    cb.evaluate(model, a.steps)
    model.save(os.path.join(out, "final_model"))


def pick_best(algo):
    res = []
    for p in sorted(glob.glob(os.path.join("models", f"{algo}_seed*", "best_suite.json"))):
        d = json.load(open(p))
        res.append((d["score_vs_SIMC"], os.path.dirname(p), d))
    if not res:
        raise SystemExit("no finished seeds found")
    res.sort(key=lambda r: r[0])
    print("\nSeed results (IAE / IAE_SIMC on the fixed suite, lower is better, 1.00 = SIMC):")
    for s, d, j in res:
        print(f"  {d}: {s:.3f}  (from SIMC {j['from_SIMC']:.3f}, 4x-low {j['from_low']:.3f}, 4x-high {j['from_high']:.3f}, "
              f"worst {j['worst']:.2f}, at {j['steps']} steps)")
    s, d, j = res[0]
    os.makedirs(os.path.join("models", f"{algo}_best"), exist_ok=True)
    shutil.copy(os.path.join(d, "best_model.zip"), os.path.join("models", f"{algo}_best", "best_model.zip"))
    shutil.copy(os.path.join(d, "best_suite.json"), os.path.join("models", f"{algo}_best", "best_suite.json"))
    print(f"-> copied {d}/best_model.zip to models/{algo}_best/best_model.zip")
    if s > 1.10:
        print("  ! the best agent is still more than 10 % worse than SIMC on the suite: train longer / more seeds.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", choices=["td3", "ddpg"], default="td3")
    ap.add_argument("--steps", type=int, default=300_000)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--gamma", type=float, default=0.99)
    ap.add_argument("--eval-every", type=int, default=20_000)
    ap.add_argument("--threads", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--source", choices=["github", "local"], default="github", help="where the step-test logs come from")
    ap.add_argument("--branch", default="main")
    ap.add_argument("--refresh", action="store_true", help="re-download the logs from GitHub")
    ap.add_argument("--skip-sysid", action="store_true", help="keep the existing models/plant_params.json")
    ap.add_argument("--pick-best", action="store_true", help="only pick the best finished seed")
    a = ap.parse_args()

    if a.pick_best:
        return pick_best(a.algo)
    if not a.skip_sysid:
        import system_id
        system_id.main(a.source, a.branch, a.refresh)        # step tests -> models/plant_params.json
    from eval_suite import reference
    ref = reference()
    print("Fixed SIMC on the suite (reference): " + ", ".join(f"{k[0]} {v['IAE']:.0f}" for k, v in ref.items() if k[1] == "SIMC"))
    for seed in a.seeds:
        train_seed(seed, a)
    pick_best(a.algo)


if __name__ == "__main__":
    main()
