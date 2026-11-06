"""
plant.py  —  shared digital twin of the rig + the exact PID the Arduino runs.

Used by: thermal_pid_env.py (RL training), tuning_rules.py / evaluate.py (classical baselines).
Parameters come from plant_params.json (written by system_id.py from the real step tests).

Plant (controlled channel = farthest thermocouple):
    τ dΔT/dt = K0 · P(t - θ) - ΔT (1 + β ΔT)        ΔT = T - T_ambient
    P = (u/100) · V² / R                              u = PWM duty 0..100 %, V = supply ("inlet") voltage
Sensor: MAX6675-like, 0.25 °C quantisation + Gaussian noise.

PID: same algorithm as the Arduino PID_v1 library (what the rig runs):
  - integral term stored as output units (ITerm += Ki*e*dt), clamped to [0,100]  -> anti-windup,
    and gain changes are bumpless (no jump when the RL agent changes Ki)
  - derivative on measurement (no derivative kick on setpoint changes)
"""
import json
import os
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def load_params(path=None):
    with open(path or os.path.join(HERE, "models", "plant_params.json")) as f:
        return json.load(f)


@dataclass
class PlantConfig:
    K0: float
    beta: float
    tau: float
    theta: float
    R: float
    T_amb: float
    noise_std: float = 0.12
    quant: float = 0.25

    @classmethod
    def nominal(cls, p=None, **over):
        p = p or load_params()
        c = cls(K0=p["K0_C_per_W"], beta=p["beta_per_C"], tau=p["tau_s"], theta=p["theta_s"],
                R=p["R_ohm_mean"], T_amb=p["T_ambient_C"])
        for k, v in over.items():
            setattr(c, k, v)
        return c

    @classmethod
    def random(cls, rng, p=None):
        p = p or load_params()
        r = p["ranges"]
        u = lambda k: float(rng.uniform(*r[k]))
        return cls(K0=u("K0"), beta=u("beta"), tau=u("tau"), theta=u("theta"), R=u("R"),
                   T_amb=u("T_ambient"), noise_std=float(rng.uniform(0.05, 0.2)))

    # steady-state temperature rise for a constant power P:  K0 P = ΔT (1 + β ΔT)
    def dT_steady(self, P):
        if self.beta < 1e-9:
            return self.K0 * P
        return (-1 + np.sqrt(1 + 4 * self.beta * self.K0 * P)) / (2 * self.beta)

    def T_max(self, V):
        return self.T_amb + self.dT_steady(V * V / self.R)

    def fopdt_at(self, V, T_op):
        """Linearised FOPDT (input = PWM %) around operating temperature T_op at supply voltage V.
        This is what classical tuning rules (ZN, Cohen-Coon, SIMC, MATLAB pidtune) are applied to."""
        dT = max(0.0, T_op - self.T_amb)
        g = 1 + 2 * self.beta * dT                      # d/dΔT of ΔT(1+βΔT)
        K_u = self.K0 * (V * V / self.R) / 100.0 / g    # °C per % PWM
        return K_u, self.tau / g, self.theta


class Plant:
    def __init__(self, cfg: PlantConfig, V: float, dt=1.0, rng=None, T0=None):
        self.cfg, self.V, self.dt = cfg, V, dt
        self.rng = rng or np.random.default_rng()
        self.dT = 0.0 if T0 is None else T0 - cfg.T_amb
        self.buf = [0.0] * max(1, int(round(cfg.theta / dt)))   # dead-time FIFO of power values

    @property
    def T(self):
        return self.cfg.T_amb + self.dT

    def measure(self):
        c = self.cfg
        y = self.T + self.rng.normal(0, c.noise_std)
        return round(y / c.quant) * c.quant

    def step(self, u):
        c = self.cfg
        self.buf.append((u / 100.0) * self.V * self.V / c.R)
        P = self.buf.pop(0)
        self.dT += self.dt * (c.K0 * P - self.dT * (1 + c.beta * self.dT)) / c.tau
        return self.measure()


@dataclass
class PID:
    """Arduino PID_v1-equivalent (DIRECT mode, output limits 0..100, sample time dt)."""
    Kp: float
    Ki: float
    Kd: float
    dt: float = 1.0
    out_min: float = 0.0
    out_max: float = 100.0
    iterm: float = 0.0
    last_y: float = None

    def set_tunings(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd     # ITerm kept -> bumpless gain change

    def __call__(self, sp, y):
        e = sp - y
        self.iterm = float(np.clip(self.iterm + self.Ki * e * self.dt, self.out_min, self.out_max))
        dy = 0.0 if self.last_y is None else (y - self.last_y) / self.dt
        self.last_y = y
        u = self.Kp * e + self.iterm - self.Kd * dy
        return float(np.clip(u, self.out_min, self.out_max))


# ----------------------------------------------------------------------------- metrics
def step_metrics(t, y, sp, t_step, band=0.02):
    """Classic step-response metrics after a setpoint step at t_step (y, sp arrays on time grid t)."""
    m = t >= t_step
    t, y, sp = t[m] - t_step, y[m], sp[m]
    y0, r = y[0], sp[-1]
    span = abs(r - y0) or 1.0
    e = sp - y
    over = max(0.0, (np.max(y) - r) / span * 100) if r >= y0 else max(0.0, (r - np.min(y)) / span * 100)
    tol = max(band * span, 0.5)                        # 2 % band, at least 0.5 °C (sensor resolution)
    outside = np.where(np.abs(e) > tol)[0]
    settle = float(t[outside[-1]]) if len(outside) else 0.0
    if len(outside) and outside[-1] == len(t) - 1:
        settle = float("inf")                          # never settled
    try:
        rise = float(t[np.argmax(y >= y0 + 0.9 * (r - y0))]) if r >= y0 else float("nan")
    except ValueError:
        rise = float("nan")
    return {"IAE": float(np.sum(np.abs(e)) * (t[1] - t[0] if len(t) > 1 else 1)),
            "overshoot_%": float(over), "settling_s": settle, "rise_s": rise,
            "ss_error": float(np.mean(np.abs(e[-120:])))}
