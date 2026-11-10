# Final rig run — setpoint 35 °C with two supply disturbances

Disturbance 1 at t = 1504 s: supply 3.26 → 5.25 V (heater full power 2.9 → 10.0 W). Disturbance 2 at t = 2650 s: → 4.06 V (6.1 W). Run length 85 min, 508 gain updates.

Twin calibrated to this run: K0 5.76 °C/W (step tests 8.51), τ 715 s (1141), θ 40 s; RMSE 0.30 °C.

| Controller | IAE total | Phase A: reach SP (min) | A: overshoot (°C) | A: settle ±0.5 (min) | Dist. 1: peak dev (°C) | Dist. 1: back in ±0.25 (min) | Dist. 2: peak dev (°C) | Dist. 2: back in ±0.25 (min) | Time in ±0.25 (%) | mean |Δu| (%/s) |
|---|---|---|---|---|---|---|---|---|---|---|
| TD3 agent — REAL RIG | 4,322 | 12.2 | 0.70 | 20.5 | 0.89 | 15.1 | 0.21 | 0.0 | 64 | 0.3 |
| TD3 agent — twin | 3,948 | 11.9 | 0.51 | 13.6 | 1.95 | 9.6 | 0.36 | 2.8 | 72 | 4.5 |
| ZN — twin | 3,771 | 12.2 | 0.21 | 11.1 | 1.63 | 18.1 | 0.24 | 0.0 | 78 | 37.3 |
| Cohen-Coon — twin | 3,693 | 12.0 | 0.25 | 11.1 | 1.63 | 14.1 | 0.30 | 2.0 | 81 | 35.3 |
| SIMC — twin | 4,040 | 11.9 | 0.65 | 17.5 | 1.65 | 9.0 | 0.32 | 3.2 | 66 | 18.1 |
| AMIGO — twin | 3,958 | 12.0 | 0.64 | 17.4 | 1.53 | 6.8 | 0.37 | 4.3 | 68 | 23.2 |
| IMC — twin | 7,074 | never | 0.00 | never | 2.25 | never | 1.18 | 4.2 | 43 | 8.6 |
| MATLAB pidtune (PI) — twin | 11,689 | never | 0.00 | never | 5.13 | never | 1.85 | 27.3 | 19 | 0.1 |
| SIMC re-tuned on the true model at every voltage (oracle) — twin | 4,581 | 11.9 | 0.51 | 15.1 | 2.47 | 13.4 | 0.37 | 5.5 | 60 | 13.2 |
