# Rig run 2 — setpoint 37 °C, two supply disturbances (scored from t1 = 3536 s)

Heater power available: 1.86 W -> 7.4 W at t1, -> 4.1 W at t2 = 5085 s. Twin calibrated to this run (from t1 - 10 min): K0 5.89 °C/W, τ 820 s, θ 40 s, RMSE 0.38 °C.

| Controller | IAE scored | D1 peak (°C) | D1 overshoot (°C) | D1 settle ±0.25 (min) | D2 peak (°C) | D2 settle ±0.25 (min) | RMS last 5 min (°C) |
|---|---|---|---|---|---|---|---|
| TD3 agent — REAL RIG | 1,037 | 2.24 | 2.24 | 12.8 | 0.77 | 9.7 | 0.097 |
| TD3 agent — twin | 937 | 1.52 | 1.52 | 22.5 | 0.58 | 4.3 | 0.053 |
| ZN — twin | 342 | 1.09 | 0.78 | 5.0 | 0.48 | 2.8 | 0.024 |
| Cohen-Coon — twin | 301 | 1.09 | 0.94 | 3.1 | 0.40 | 2.1 | 0.022 |
| SIMC — twin | 813 | 1.60 | 1.60 | 13.6 | 0.53 | 6.8 | 0.066 |
| AMIGO — twin | 712 | 1.61 | 1.61 | 8.3 | 0.56 | 6.0 | 0.052 |
| IMC — twin | 4,913 | 4.28 | 4.28 | never | 1.76 | never | 0.247 |
| MATLAB pidtune (PI) — twin | 8,106 | 6.75 | 6.75 | never | 1.97 | never | 1.384 |
