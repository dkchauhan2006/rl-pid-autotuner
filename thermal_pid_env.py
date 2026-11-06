"""
thermal_pid_env.py  —  Gymnasium environment: RL agent tunes the Arduino PID live.

The fast loop (PID_v1, every 1 s) controls the heater; the agent (slow loop, every RL_EVERY s)
nudges Kp, Ki, Kd — exactly the two-speed design in the proposal.

Plant = digital twin fitted from the real step tests (plant.py / plant_params.json),
domain-randomised every episode (K0, β, τ, θ, R, ambient, sensor noise) so the policy transfers to the rig.

Each episode (90 min of plant time) contains:
  * a setpoint step (cold start, or from a warm steady state),
  * at a random time, a DISTURBANCE: the supply ("inlet") voltage changes and/or the setpoint changes.
The agent starts from random gains, so it must learn to fix bad tuning as well as adapt.

Observation (all roughly in [-1, 1]):
  0 e/10                      error (°C)
  1 slope                     dT/dt over the last RL window (°C per 10 s)
  2 u_mean/100 - 0.5          mean PWM over the window
  3 u_chatter/50              mean |Δu| per second over the window (actuator wear)
  4..6 z_Kp, z_Ki, z_Kd       current gains, log-scaled to [-1, 1] within the safety bounds
  7 (V - 3.5)/2               measured supply voltage
  8 headroom                  (SP - T_amb)/(T_max(V) - T_amb): how hard the heater must work
  9 dT/50                     temperature above ambient
Action: [dz_Kp, dz_Ki, dz_Kd] in [-1, 1]  ->  gains move multiplicatively (log space), clipped to bounds.
Reward (per second, averaged over the window):
  -|e|/2  - 2·max(0, T-SP)/2 (overshoot)  - LAMBDA_DU·|Δu|/100 (chatter)  - small gain-change cost
"""
import numpy as np
import gymnasium as gym
from gymnasium import spaces

from plant import Plant, PlantConfig, PID, load_params

RL_EVERY = 10                 # seconds between agent decisions (PID runs every 1 s)
EPISODE_S = 5400              # 90 minutes of plant time per episode (τ ≈ 15–20 min)
BOUNDS = {                    # hard safety bounds on the gains (PID_v1 parallel form)
    "Kp": (1.0, 300.0),
    "Ki": (0.002, 3.0),
    "Kd": (1.0, 3000.0),
}
LAMBDA_DU = 1.0
STEP_Z = 0.06                 # max gain move per decision in normalised log units (range is 2)


def z_to_gain(z, name):
    lo, hi = BOUNDS[name]
    return float(np.exp(np.log(lo) + (np.clip(z, -1, 1) + 1) / 2 * (np.log(hi) - np.log(lo))))


def gain_to_z(g, name):
    lo, hi = BOUNDS[name]
    return float(np.clip(2 * (np.log(np.clip(g, lo, hi)) - np.log(lo)) / (np.log(hi) - np.log(lo)) - 1, -1, 1))


class ThermalPIDEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, randomize=True, scenario=None, seed=None, init_gains=None, V_measured=True):
        super().__init__()
        self.params = load_params()
        self.randomize, self.scenario, self.init_gains = randomize, scenario, init_gains
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
        kind = self.rng.choice(["voltage", "setpoint", "both", "none"], p=[0.45, 0.25, 0.2, 0.1])
        if kind in ("voltage", "both"):
            while abs(V2 - V1) < 0.8:
                V2 = float(self.rng.uniform(*r["V"]))
        Vmin = min(V1, V2)
        head = lambda: float(self.rng.uniform(0.3, 0.85))
        sp1 = cfg.T_amb + head() * (cfg.T_max(Vmin) - cfg.T_amb)       # reachable at both voltages
        sp2 = sp1 if kind in ("voltage", "none") else cfg.T_amb + head() * (cfg.T_max(Vmin) - cfg.T_amb)
        warm = bool(self.rng.random() < 0.35)                          # start already at a steady state
        T0 = cfg.T_amb if not warm else cfg.T_amb + float(self.rng.uniform(0.2, 0.9)) * (sp1 - cfg.T_amb)
        t_event = int(self.rng.uniform(1800, 3600))
        return dict(cfg=cfg, V1=V1, V2=V2, sp1=sp1, sp2=sp2, T0=T0, t_event=t_event, kind=str(kind))

    # ------------------------------------------------------------------ helpers
    def _gains(self):
        return z_to_gain(self.z[0], "Kp"), z_to_gain(self.z[1], "Ki"), z_to_gain(self.z[2], "Kd")

    def _obs(self):
        c, sc = self.cfg, self.sc
        sp = self.sp
        T = self.meas
        V = self.plant.V if self.V_measured else 3.5
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
        self.plant = Plant(self.cfg, sc["V1"], rng=self.rng, T0=sc["T0"])
        if self.init_gains is not None:
            g = self.init_gains
            self.z = np.array([gain_to_z(g["Kp"], "Kp"), gain_to_z(g["Ki"], "Ki"), gain_to_z(g["Kd"], "Kd")])
        else:
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
        self.sp = sc["sp1"]
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
