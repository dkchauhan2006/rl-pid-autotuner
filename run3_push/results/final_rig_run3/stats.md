# Rig run 3 — setpoint 36 °C, cold start + one supply disturbance (t1 = 4097 s)

Heater power available: 3.55 W -> 6.38 W at t1. Ambient 24.36 °C. Twin calibrated on the whole run: K0 6.18 °C/W, τ 856 s, θ 40 s, RMSE 0.26 °C.

| Controller | IAE total | reach SP (min) | overshoot (°C) | settle ±0.25 (min) | hold RMS 15 min→t1 (°C) | D1 peak (°C) | D1 back ±0.25 (min) | D1 back ±0.1 (min) | RMS last 5 min (°C) | mean abs du (%/s) |
|---|---|---|---|---|---|---|---|---|---|---|
| TD3 agent — REAL RIG | 5,096 | 13.4 | 0.72 | 64.5 | 0.211 | 0.56 | 4.7 | 9.3 | 0.031 | 0.4 |
| TD3 agent — twin | 4,926 | 13.3 | 0.61 | never | 0.202 | 0.50 | 3.4 | 7.5 | 0.076 | 3.6 |
| ZN — twin | 4,550 | 13.6 | 0.32 | 16.8 | 0.080 | 0.44 | 2.3 | 19.9 | 0.058 | 44.7 |
| Cohen-Coon — twin | 4,505 | 13.4 | 0.33 | 16.0 | 0.066 | 0.43 | 2.0 | 11.7 | 0.035 | 41.6 |
| SIMC — twin | 4,917 | 13.4 | 0.86 | 22.1 | 0.236 | 0.44 | 7.0 | 12.3 | 0.036 | 18.3 |
| AMIGO — twin | 4,857 | 13.4 | 0.85 | 21.4 | 0.224 | 0.43 | 6.4 | 9.2 | 0.056 | 24.3 |
| IMC — twin | 6,869 | 32.0 | 0.16 | 28.7 | 0.382 | 1.45 | never | never | 0.809 | 8.4 |
| MATLAB pidtune (PI) — twin | 10,325 | 26.8 | 0.66 | 67.7 | 0.788 | 2.33 | never | never | 1.324 | 0.1 |
