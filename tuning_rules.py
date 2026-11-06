"""
tuning_rules.py  —  classical PID tuning rules from a FOPDT model (K, τ, θ).

These are the baselines the RL agent must match (at the voltage they were tuned for) and beat
(when the supply voltage changes). Returned gains are in PID_v1 parallel form:
    u = Kp e + Ki ∫e dt + Kd de/dt      (Ki = Kp/Ti, Kd = Kp·Td)

  ZN       : Ziegler–Nichols open-loop (reaction curve) rule
  CC       : Cohen–Coon
  SIMC     : Skogestad IMC (τc = θ)  — close to what MATLAB pidtune gives for FOPDT lag-dominant plants
  AMIGO    : Åström–Hägglund AMIGO PID
  IMC      : IMC/lambda tuning with λ = max(0.25τ, 1.5θ) (robust)

MATLAB's own pidtune: see matlab/pidtune_compare.m (run it in MATLAB and paste the gains into
BASELINE_OVERRIDES in evaluate.py for an exact comparison).
"""


def _pid(Kp, Ti, Td):
    return {"Kp": Kp, "Ki": Kp / Ti if Ti > 0 else 0.0, "Kd": Kp * Td}


def zn(K, tau, theta):
    return _pid(1.2 * tau / (K * theta), 2.0 * theta, 0.5 * theta)


def cohen_coon(K, tau, theta):
    r = theta / tau
    Kp = (tau / (K * theta)) * (4 / 3 + r / 4)
    Ti = theta * (32 + 6 * r) / (13 + 8 * r)
    Td = theta * 4 / (11 + 2 * r)
    return _pid(Kp, Ti, Td)


def simc(K, tau, theta, tau_c=None):
    tau_c = theta if tau_c is None else tau_c
    # SIMC PI (serial) + derivative for lag-dominant: Kc = τ/(K(τc+θ)), Ti = min(τ, 4(τc+θ))
    Kp = tau / (K * (tau_c + theta))
    Ti = min(tau, 4 * (tau_c + theta))
    Td = theta / 3                       # mild derivative (θ/3), as commonly paired with SIMC
    return _pid(Kp, Ti, Td)


def amigo(K, tau, theta):
    Kp = (1 / K) * (0.2 + 0.45 * tau / theta)
    Ti = theta * (0.4 * theta + 0.8 * tau) / (theta + 0.1 * tau)
    Td = 0.5 * theta * tau / (0.3 * theta + tau)
    return _pid(Kp, Ti, Td)


def imc(K, tau, theta, lam=None):
    lam = max(0.25 * tau, 1.5 * theta) if lam is None else lam
    Kp = (tau + theta / 2) / (K * (lam + theta / 2))
    Ti = tau + theta / 2
    Td = tau * theta / (2 * tau + theta)
    return _pid(Kp, Ti, Td)


RULES = {"ZN": zn, "Cohen-Coon": cohen_coon, "SIMC": simc, "AMIGO": amigo, "IMC": imc}


def all_rules(K, tau, theta):
    return {name: f(K, tau, theta) for name, f in RULES.items()}
