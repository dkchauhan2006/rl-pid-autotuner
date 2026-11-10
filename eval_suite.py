"""
eval_suite.py  —  FIXED test suite used to pick the best checkpoint (train_td3.py) and to report results.

Same scenarios, same plant, same noise seed every time, so two checkpoints are compared fairly
(v1 picked the model with the best mean reward over 8 RANDOM episodes: ±100–250 noise, so "best" was luck).

Score = mean over episodes of  IAE(agent) / IAE(SIMC)  on the same scenario (1.00 = classical SIMC accuracy),
        + 0.1 × the extra heater chatter above SIMC's (so "more aggressive" is not rewarded for free).
Starts: from SIMC gains, from gains 4x too LOW, from gains 4x too HIGH (the agent must repair those).
"""
import numpy as np

from plant import PlantConfig, load_params
from thermal_pid_env import NAMES, ThermalPIDEnv, z_neutral

STARTS = {
    "SIMC": [z_neutral(n) for n in NAMES],
    "low": [-0.9, -0.9, z_neutral("Kd")],          # Kp, Ki ≈ 0.28× SIMC: sluggish
    "high": [0.9, 0.9, 1.0],                        # Kp, Ki ≈ 3.5× SIMC, Kd 2×: aggressive
}


def scenarios(cfg):
    sp = lambda V, h: cfg.T_amb + h * (cfg.T_max(V) - cfg.T_amb)
    S = {
        "cold_2V": dict(V1=2.0, V2=2.0, sp1=sp(2.0, .6), sp2=sp(2.0, .6), T0=cfg.T_amb, t_event=10**9),
        "cold_3V": dict(V1=3.0, V2=3.0, sp1=sp(3.0, .6), sp2=sp(3.0, .6), T0=cfg.T_amb, t_event=10**9),
        "cold_4V": dict(V1=4.0, V2=4.0, sp1=sp(4.0, .6), sp2=sp(4.0, .6), T0=cfg.T_amb, t_event=10**9),
        "cold_5V": dict(V1=5.0, V2=5.0, sp1=sp(5.0, .6), sp2=sp(5.0, .6), T0=cfg.T_amb, t_event=10**9),
        "cold_4V_SP40": dict(V1=4.0, V2=4.0, sp1=40.0, sp2=40.0, T0=cfg.T_amb, t_event=10**9),
        "supply_drop_5to3V": dict(V1=5.0, V2=3.0, sp1=sp(3.0, .75), sp2=sp(3.0, .75), T0=sp(3.0, .75), t_event=900),
        "supply_rise_3to5V": dict(V1=3.0, V2=5.0, sp1=sp(3.0, .75), sp2=sp(3.0, .75), T0=sp(3.0, .75), t_event=900),
        "sp_step_4V": dict(V1=4.0, V2=4.0, sp1=sp(4.0, .4), sp2=sp(4.0, .75), T0=sp(4.0, .4), t_event=900),
    }
    for k, v in S.items():
        v.update(cfg=cfg, kind=k)
    return S


def run_episode(sc, init_z, policy=None, seed=0):
    env = ThermalPIDEnv(randomize=False, scenario=sc, seed=seed, init_z=init_z)
    obs, _ = env.reset(seed=seed)
    done = False
    while not done:
        a = policy(obs) if policy is not None else np.zeros(3)
        obs, _, te, tr, _ = env.step(a)
        done = te or tr
    tr = env.trajectory()
    e = tr[:, 2] - tr[:, 1]
    tail = e[-600:]
    return dict(IAE=float(np.sum(np.abs(e))), overshoot=float(max(0.0, np.max(tr[:, 1] - tr[:, 2]))),
                ss_err=float(np.mean(np.abs(tail))), chatter=float(np.mean(np.abs(np.diff(tr[:, 3])))),
                Kp_end=float(tr[-1, 5]), Ki_end=float(tr[-1, 6]), Kd_end=float(tr[-1, 7])), tr


_REF = {}


def reference(cfg_key="nominal"):
    """IAE of fixed SIMC (and of the fixed wrong gains, no agent) on every scenario. Cached."""
    if cfg_key not in _REF:
        cfg = PlantConfig.nominal(load_params())
        S = scenarios(cfg)
        _REF[cfg_key] = {(sn, st): run_episode(sc, STARTS[st])[0] for sn, sc in S.items() for st in STARTS}
    return _REF[cfg_key]


def score(policy):
    """Returns (score, details). score = mean IAE(agent)/IAE(fixed SIMC) over all scenarios and starts."""
    cfg = PlantConfig.nominal(load_params())
    S = scenarios(cfg)
    ref = reference()
    rows = []
    for sn, sc in S.items():
        simc_iae = ref[(sn, "SIMC")]["IAE"]
        for st, z0 in STARTS.items():
            m, _ = run_episode(sc, z0, policy)
            rows.append(dict(scenario=sn, start=st, ratio=m["IAE"] / simc_iae,
                             chatter_ratio=m["chatter"] / max(ref[(sn, "SIMC")]["chatter"], 1e-6),
                             fixed_ratio=ref[(sn, st)]["IAE"] / simc_iae, **m))
    s = np.mean([r["ratio"] for r in rows]) + 0.1 * np.mean([max(0.0, r["chatter_ratio"] - 1) for r in rows])
    return float(s), rows
