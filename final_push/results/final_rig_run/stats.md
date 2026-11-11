# Final rig run — setpoint 35 °C with two supply disturbances

Disturbance 1 at t = 1504 s: supply 3.26 → 5.25 V (heater full power 2.9 → 9.3 W). Disturbance 2 at t = 2650 s: → 4.06 V (5.5 W). Run length 85 min, 508 gain updates.

Twin calibrated to this run: K0 6.29 °C/W (step tests 8.51), τ 837 s (1141), θ 40 s; RMSE 0.19 °C.

| Controller | IAE total | Phase A: reach SP (min) | A: overshoot (°C) | A: settle ±0.5 (min) | Dist. 1: peak dev (°C) | Dist. 1: back in ±0.25 (min) | Dist. 2: peak dev (°C) | Dist. 2: back in ±0.25 (min) | Time in ±0.25 (%) | mean |Δu| (%/s) |
|---|---|---|---|---|---|---|---|---|---|---|
| TD3 agent — REAL RIG | 4,322 | 12.2 | 0.70 | 20.5 | 0.89 | 15.1 | 0.21 | 0.0 | 64 | 0.3 |
| TD3 agent — twin | 4,038 | 12.1 | 0.56 | 14.1 | 1.20 | 8.9 | 0.33 | 2.8 | 69 | 3.7 |
| ZN — twin | 3,837 | 12.3 | 0.25 | 11.3 | 1.26 | 18.1 | 0.24 | 0.0 | 79 | 37.8 |
| Cohen-Coon — twin | 3,728 | 12.1 | 0.31 | 11.2 | 1.24 | 9.2 | 0.27 | 2.0 | 82 | 35.6 |
| SIMC — twin | 4,161 | 12.1 | 0.75 | 18.6 | 1.32 | 8.7 | 0.30 | 3.8 | 65 | 18.0 |
| AMIGO — twin | 4,072 | 12.1 | 0.74 | 18.4 | 1.22 | 6.8 | 0.36 | 4.7 | 66 | 23.2 |
| IMC — twin | 6,982 | never | 0.00 | never | 2.16 | never | 1.17 | 4.4 | 43 | 8.6 |
| MATLAB pidtune (PI) — twin | 11,779 | never | 0.00 | never | 4.94 | never | 1.99 | 30.7 | 15 | 0.1 |
| SIMC re-tuned on the true model at every voltage (oracle) — twin | 4,478 | 12.1 | 0.56 | 15.9 | 1.94 | 12.3 | 0.33 | 4.9 | 62 | 15.0 |
