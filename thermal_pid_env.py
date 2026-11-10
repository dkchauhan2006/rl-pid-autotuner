"""
thermal_pid_env.py  —  Gymnasium environment: RL agent tunes the Arduino PID live.

The fast loop (PID_v1, every 1 s) controls the heater; the agent (slow loop, every RL_EVERY s)
nudges Kp, Ki, Kd — the two-speed design in the proposal.

Plant = digital twin fitted from the real step tests (plant.py / plant_params.json),
domain-randomised every episode (K0, β, τ, θ, R, ambient, sensor noise/resolution) so the policy transfers to the rig.

WHAT CHANGED (v2) AND WHY — the v1 agent pinned its gains to the bounds (Kp→300, Ki→0.002, Kd→1):
  1. Gains are ANCHORED to the model-based SIMC gains at the measured supply voltage and setpoint:
         gain = SIMC_gain(V, SP) × multiplier,   multiplier in [0.25, 4] (Kp, Ki) and [0.05, 2] (Kd)
     v1 let every gain roam over 2.5 decades (Kp 1–300, Ki 0.002–3) and the agent learnt a bang-bang corner
     with NO integral action (steady-state offset). Now the integral can never be switched off (Ki ≥ 0.25×SIMC)
     and zero action = classical SIMC accuracy, so the agent can only refine or repair the tuning.
  2. Episodes start from SIMC ± noise (40 %) or from wrong gains anywhere in the 16× range (60 %).
     v1 started 100 % from uniformly random gains over the huge range, mostly absurd ones.
  3. Sensor noise/resolution randomised to cover the NTC now on the rig (v1: thermocouple 0.25 °C only).
  4. (train_td3.py) γ 0.98 → 0.99 (the plant needs ~1000 s to show the effect of a gain change; 0.98 looked only
     ~500 s ahead), longer training, several seeds, and the best model is picked on a FIXED test suite against
     SIMC instead of a noisy mean reward over 8 random episodes.

Each episode (90 min of plant time) contains:
  * a setpoint step (cold start, or from a warm steady state),
  * at a random time, a DISTURBANCE: the supply voltage changes and/or the setpoint changes.

Observation (all roughly in [-1, 1]):
  0 e/10                      error (°C)
  1 slope                     dT/dt over the last RL window (°C per 10 s)
  2 u_mean/100 - 0.5          mean PWM over the window
  3 u_chatter/50              mean |Δu| per second over the window (actuator wear)
  4..6 z_Kp, z_Ki, z_Kd       current gain MULTIPLIERS (relative to SIMC), log-scaled to [-1, 1]
  7 (V - 3.5)/2               measured supply voltage
  8 headroom                  (SP - T_amb)/(T_max(V) - T_amb): how hard the heater must work
  9 dT/50                     temperature above ambient
Action: [dz_Kp, dz_Ki, dz_Kd] in [-1, 1]  ->  multipliers move in log space by at most STEP_Z per decision.
Reward (per second, averaged over the window):
  -|e|/2  - 2·max(0, T-SP)/2 (overshoot)  - LAMBDA_DU·|Δu|/100 (chatter)  - small gain-change cost
"""
import numpy as np
import gymnasium as gym
from gymnasium import spaces

from plant import Plant, PlantConfig, PID, load_params
from tuning_rules import simc

RL_EVERY = 10                 # seconds between agent decisions (PID runs every 1 s)
EPISODE_S = 5400              # 90 minutes of plant time per episode (τ ≈ 15–20 min)
MULT = {                      # allowed multiplier of the SIMC gain (log-scaled to z in [-1, 1]; z = 0 -> ×1 for Kp, Ki)
    "Kp": (0.25, 4.0),
    "Ki": (0.25, 4.0),
    "Kd": (0.05, 2.0),        # derivative on a noisy sensor: allow much less D, at most 2× SIMC's θ/3
}
SAFETY = {                    # absolute hard limits = the firmware's clamp (arduino_nano_rl_pid.ino)
    "Kp": (0.5, 1000.0),
    "Ki": (0.0005, 10.0),
    "Kd": (0.0, 5000.0),
}
NAMES = ("Kp", "Ki", "Kd")
LAMBDA_DU = 1.0
STEP_Z = 0.06                 # max multiplier move per decision in normalised log units (range is 2)


def z_neutral(name):
    """z that gives multiplier 1 (SIMC itself)."""
    lo, hi = MULT[name]
    return float(2 * (0 - np.log(lo)) / (np.log(hi) - np.log(lo)) - 1)


def z_to_mult(z, name):
    lo, hi = MULT[name]
    return float(np.exp(np.log(lo) + (np.clip(z, -1, 1) + 1) / 2 * (np.log(hi) - np.log(lo))))


def mult_to_z(m, name):
    lo, hi = MULT[name]
    return float(np.clip(2 * (np.log(np.clip(m, lo, hi)) - np.log(lo)) / (np.log(hi) - np.log(lo)) - 1, -1, 1))


def base_gains(cfg, V, sp):
    """Model-based SIMC gains for supply V and setpoint sp (the anchor). Same function on the rig (live_tuner.py)."""
    return simc(*cfg.fopdt_at(V, sp))


def gains_from_z(z, base):
    return tuple(float(np.clip(base[n] * z_to_mult(z[i], n), *SAFETY[n])) for i, n in enumerate(NAMES))


def z_from_gains(g, base):
    """g: dict or (Kp, Ki, Kd). Gains outside the allowed range are clipped to it."""
    vals = [g[n] for n in NAMES] if isinstance(g, dict) else list(g)
    return np.array([mult_to_z(max(v, 1e-9) / max(base[n], 1e-9), n) for v, n in zip(vals, NAMES)])


class ThermalPIDEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, randomize=True, scenario=None, seed=None, init_gains=None, init_z=None, V_measured=True):
        super().__init__()
        self.params = load_params()
        self.randomize, self.scenario = randomize, scenario
        self.init_gains, self.init_z = init_gains, init_z
        self.V_measured = V_measured
        self.rng = np.random.default_rng(seed)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(-5.0, 5.0, shape=(10,), dtype=np.float32)

    # ------------------------------------------------------------------ scenario generation
    def _sample_scenario(self):
        r = self.params["ranges"]
        cfg = PlantConfig.random(self.rng, self.params) if self.randomize else PlantConfig.nominal(self.params)
        V1 = float(self.rng.uniform(*r["V"]))
        V2 = V1
        kind = self.rng.choice(["voltage", "setpoint", "both", "none"], p=[0.35, 0.3, 0.2, 0.15])
        if kind in ("voltage", "both"):
            while abs(V2 - V1) < 0.8:
                V2 = float(self.rng.uniform(*r["V"]))
        Vmin = min(V1, V2)
        head = lambda: float(self.rng.uniform(0.25, 0.85))
        sp1 = cfg.T_amb + head() * (cfg.T_max(Vmin) - cfg.T_amb)       # reachable at both voltages
        sp2 = sp1 if kind in ("voltage", "none") else cfg.T_amb + head() * (cfg.T_max(Vmin) - cfg.T_amb)
        warm = bool(self.rng.random() < 0.35)                          # start already at a steady state
        T0 = cfg.T_amb if not warm else cfg.T_amb + float(self.rng.uniform(0.2, 0.9)) * (sp1 - cfg.T_amb)
        t_event = int(self.rng.uniform(1800, 3600))
        return dict(cfg=cfg, V1=V1, V2=V2, sp1=sp1, sp2=sp2, T0=T0, t_event=t_event, kind=str(kind))

    # ------------------------------------------------------------------ helpers
    def _V_seen(self):
        return self.plant.V if self.V_measured else 3.5

    def _update_base(self):
        # the controller knows only the NOMINAL model (as on the rig), not the randomised plant
        self.base = base_gains(self.nominal, self._V_seen(), self.sp)

    def _gains(self):
        return gains_from_z(self.z, self.base)

    def _obs(self):
        c = self.cfg
        sp, T = self.sp, self.meas
        V = self._V_seen()
        head = (sp - c.T_amb) / max(1e-3, c.T_max(V) - c.T_amb)
        return np.array([
            (sp - T) / 10.0,
            np.clip((T - self.T_prev_window), -10, 10) / 1.0,
            self.u_mean / 100.0 - 0.5,
            self.chatter / 50.0,
            *self.z,
            (V - 3.5) / 2.0,
            np.clip(head, 0, 2),
            (T - c.T_amb) / 50.0,
        ], dtype=np.float32)

    # ------------------------------------------------------------------ gym API
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.sc = self.scenario if self.scenario is not None else self._sample_scenario()
        sc = self.sc
        self.cfg = sc["cfg"]
        self.nominal = PlantConfig.nominal(self.params)
        self.plant = Plant(self.cfg, sc["V1"], rng=self.rng, T0=sc["T0"])
        self.sp = sc["sp1"]
        self._update_base()
        if self.init_z is not None:
            self.z = np.clip(np.asarray(self.init_z, dtype=float), -1, 1)
        elif self.init_gains is not None:
            self.z = z_from_gains(self.init_gains, self.base)
        elif self.rng.random() < 0.4:                                   # near the classical tuning
            self.z = np.clip([z_neutral(n) for n in NAMES] + self.rng.normal(0, 0.15, 3), -1, 1)
        else:                                                           # wrong tuning: up to 4x off
            self.z = self.rng.uniform(-1, 1, size=3)
        self.pid = PID(*self._gains())
        # warm start: integral preloaded to the steady output so we start in equilibrium
        if sc["T0"] > self.cfg.T_amb + 0.5:
            dT = sc["T0"] - self.cfg.T_amb
            P = dT * (1 + self.cfg.beta * dT) / self.cfg.K0
            u_ss = float(np.clip(100 * P / (sc["V1"] ** 2 / self.cfg.R), 0, 100))
            self.pid.iterm, self.pid.last_y = u_ss, sc["T0"]
            self.plant.buf = [(u_ss / 100) * sc["V1"] ** 2 / self.cfg.R] * len(self.plant.buf)
        self.t = 0
        self.meas = self.plant.measure()
        self.T_prev_window = self.meas
        self.u_last = self.pid.iterm
        self.u_mean, self.chatter = self.u_last, 0.0
        self.log = []
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64), -1, 1)
        self.z = np.clip(self.z + STEP_Z * a, -1, 1)
        self.pid.set_tunings(*self._gains())

        sc = self.sc
        r_sum, us, dus = 0.0, [], []
        self.T_prev_window = self.meas
        for _ in range(RL_EVERY):
            if self.t == sc["t_event"]:
                self.plant.V = sc["V2"]
                self.sp = sc["sp2"]
                self._update_base()                    # gain schedule follows the measured voltage / new setpoint
                self.pid.set_tunings(*self._gains())
            u = self.pid(self.sp, self.meas)
            self.meas = self.plant.step(u)
            T = self.plant.T                           # true temperature for the reward (noise-free)
            e = self.sp - T
            du = abs(u - self.u_last)
            r_sum += -abs(e) / 2.0 - 2.0 * max(0.0, -e) / 2.0 - LAMBDA_DU * du / 100.0
            us.append(u); dus.append(du)
            self.u_last = u
            self.log.append((self.t, T, self.sp, u, self.plant.V, *self._gains()))
            self.t += 1
        reward = r_sum / RL_EVERY - 0.02 * float(np.sum(np.abs(a)))
        self.u_mean, self.chatter = float(np.mean(us)), float(np.mean(dus))

        terminated = bool(self.plant.T > 150)          # runaway guard (never reachable with V ≤ 5.5 V)
        truncated = self.t >= EPISODE_S
        if terminated:
            reward -= 100.0
        return self._obs(), float(reward), terminated, truncated, {}

    def trajectory(self):
        """(t, T, SP, u, V, Kp, Ki, Kd) arrays of the episode so far, for plots and metrics."""
        return np.array(self.log)
