"""
thermal_pid_env.py

Digital-twin simulator of the Arduino heater + thermocouple rig, wrapped as a
Gymnasium environment. The RL agent (TD3) trains here -- NOT on the real
hardware -- because it needs hundreds of thousands of interactions.

Physics model: First Order Plus Dead Time (FOPDT)
    tau * dT/dt = -(T - T_ambient) + K * u(t - theta)

K, tau, theta are the parameters you get from fit_fopdt.py after running a
real step test on your rig. We don't train on a single fixed (K, tau, theta) --
instead we RANDOMIZE them every episode within a range around your fitted
values ("domain randomization"). This makes the trained agent robust to the
real plant being slightly different from the fitted model (the "reality gap").

State (observation):  [error, delta_error, last_u, Kp, Ki, Kd]
Action:                [dKp, dKi, dKd]  (normalized -1..1, scaled internally)
Reward:                -|error| - lambda*(delta_u)^2 - small ITAE term
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces


class ThermalPIDEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        dt_pid=0.5,                      # seconds between PID/RL decisions
        episode_seconds=300,             # length of one training episode
        K_range=(1.5, 3.5),              # process gain range -- REPLACE with your fit_fopdt.py output
        tau_range=(40, 90),               # time constant range (s) -- REPLACE
        theta_range=(1, 5),               # dead time range (s) -- REPLACE
        kp_bounds=(0.1, 15.0),           # HARD safety bounds on gains
        ki_bounds=(0.001, 2.0),
        kd_bounds=(0.0, 5.0),
        T_ambient=25.0,
        setpoint_range=(50, 120),
        action_scale=np.array([1.0, 0.05, 0.2], dtype=np.float32),  # max gain change per RL step
        du_penalty=0.01,
        itae_weight=0.001,
    ):
        super().__init__()
        self.dt = dt_pid
        self.episode_steps = int(episode_seconds / dt_pid)
        self.K_range, self.tau_range, self.theta_range = K_range, tau_range, theta_range
        self.kp_bounds, self.ki_bounds, self.kd_bounds = kp_bounds, ki_bounds, kd_bounds
        self.T_ambient = T_ambient
        self.setpoint_range = setpoint_range
        self.action_scale = action_scale
        self.du_penalty = du_penalty
        self.itae_weight = itae_weight

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)

    # ---------- internal helpers ----------
    def _randomize_plant(self):
        self.K = np.random.uniform(*self.K_range)
        self.tau = np.random.uniform(*self.tau_range)
        self.theta = np.random.uniform(*self.theta_range)
        self.delay_steps = max(1, int(round(self.theta / self.dt)))
        self.u_buffer = [0.0] * (self.delay_steps + 1)  # dead-time FIFO buffer

    def _get_obs(self):
        error = self.setpoint - self.T
        d_error = error - self.prev_error
        return np.array([error, d_error, self.last_u, self.Kp, self.Ki, self.Kd], dtype=np.float32)

    # ---------- gym API ----------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._randomize_plant()
        self.T = self.T_ambient + np.random.uniform(-2, 2)
        self.setpoint = np.random.uniform(*self.setpoint_range)
        # start from a random-ish but reasonable gain set so agent learns to correct bad tuning too
        self.Kp = np.random.uniform(*self.kp_bounds)
        self.Ki = np.random.uniform(0, self.ki_bounds[1] * 0.5)
        self.Kd = np.random.uniform(*self.kd_bounds)
        self.integral = 0.0
        self.prev_error = self.setpoint - self.T
        self.last_u = 0.0
        self.t = 0
        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        # 1) RL updates the gains (clipped to hard safety bounds -- this is
        #    the simple safety mechanism; see README for the full
        #    inverting-gradient version mentioned in the proposal)
        action = np.clip(action, -1.0, 1.0) * self.action_scale
        self.Kp = float(np.clip(self.Kp + action[0], *self.kp_bounds))
        self.Ki = float(np.clip(self.Ki + action[1], *self.ki_bounds))
        self.Kd = float(np.clip(self.Kd + action[2], *self.kd_bounds))

        # 2) discrete PID, position form, with anti-windup clamp
        error = self.setpoint - self.T
        self.integral = np.clip(self.integral + error * self.dt, -50.0, 50.0)
        d_error = (error - self.prev_error) / self.dt
        u = self.Kp * error + self.Ki * self.integral + self.Kd * d_error
        u = float(np.clip(u, 0.0, 100.0))  # heater output 0-100%

        # 3) plant dynamics: FOPDT, Euler integration
        self.u_buffer.append(u)
        u_delayed = self.u_buffer.pop(0)
        dTdt = (-(self.T - self.T_ambient) + self.K * u_delayed) / self.tau
        self.T = self.T + dTdt * self.dt

        # 4) reward
        du = u - self.last_u
        itae_term = self.t * self.dt * abs(error)
        reward = -abs(error) - self.du_penalty * (du ** 2) - self.itae_weight * itae_term

        self.last_u = u
        self.prev_error = error
        self.t += 1

        terminated = False
        truncated = self.t >= self.episode_steps
        if self.T > 250 or self.T < -10:   # runaway / sensor-fault guard
            reward -= 100.0
            terminated = True

        obs = self._get_obs()
        info = {"T": self.T, "u": u, "Kp": self.Kp, "Ki": self.Ki, "Kd": self.Kd, "setpoint": self.setpoint}
        return obs, reward, terminated, truncated, info
