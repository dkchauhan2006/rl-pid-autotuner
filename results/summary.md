# Results: RL agent vs classical PID tuning

Plant: digital twin of the rig, farthest sensor (ch1). Classical gains tuned once at 4 V. Same plant, scenario and noise seed for every controller.

## Classical gains (tuned at 4 V)

| rule | Kp | Ki | Kd |
|---|---|---|---|
| ZN | 93.96 | 1.1745 | 1879.2 |
| Cohen-Coon | 105.22 | 1.0878 | 1518.9 |
| SIMC | 39.15 | 0.1223 | 522.0 |
| AMIGO | 35.89 | 0.1561 | 708.9 |
| IMC | 12.43 | 0.0128 | 243.4 |
| MATLAB pidtune | 4.70 | 0.0091 | 0.0 |

## cold_2V

| controller | IAE (°C·s) | overshoot % | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 3527 | 3.6 | 1387 | 0.04 | 47.4 |
| Cohen-Coon | 3207 | 3.1 | 1321 | 0.04 | 46.2 |
| SIMC | 3189 | 5.5 | 1154 | 0.01 | 45.3 |
| AMIGO | 3582 | 6.0 | 1324 | 0.02 | 47.0 |
| IMC | 5243 | 0.6 | 2253 | 0.02 | 46.1 |
| MATLAB pidtune | 7244 | 3.4 | 2637 | 0.16 | 0.9 |
| ZN re-tuned (oracle) | 2938 | 1.1 | 1191 | 0.04 | 45.0 |
| RL (from bad gains) | 3944 | 0.0 | 2429 | 0.17 | 20.2 |
| RL (from ZN gains) | 2659 | 2.2 | 917 | 0.04 | 32.6 |

## cold_3V

| controller | IAE (°C·s) | overshoot % | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 5751 | 2.1 | 1218 | 0.03 | 45.9 |
| Cohen-Coon | 5485 | 2.0 | 1164 | 0.04 | 44.7 |
| SIMC | 5437 | 4.4 | 1040 | 0.01 | 43.3 |
| AMIGO | 5759 | 4.4 | 1163 | 0.01 | 44.3 |
| IMC | 6637 | 3.9 | 1236 | 0.01 | 41.8 |
| MATLAB pidtune | 9238 | 7.5 | 3482 | 0.02 | 0.9 |
| ZN re-tuned (oracle) | 5835 | 1.7 | 1265 | 0.01 | 45.8 |
| RL (from bad gains) | 6604 | 0.8 | 2353 | 0.17 | 15.2 |
| RL (from ZN gains) | 5119 | 3.3 | 895 | 0.03 | 15.5 |

## cold_4V

| controller | IAE (°C·s) | overshoot % | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 8261 | 2.3 | 1049 | 0.02 | 42.7 |
| Cohen-Coon | 8002 | 1.6 | 959 | 0.03 | 42.7 |
| SIMC | 8144 | 3.2 | 1371 | 0.06 | 42.1 |
| AMIGO | 8256 | 3.7 | 1426 | 0.06 | 43.4 |
| IMC | 10682 | 9.3 | 3148 | 0.03 | 39.8 |
| MATLAB pidtune | 10420 | 8.5 | 2822 | 0.02 | 0.9 |
| ZN re-tuned (oracle) | 8261 | 2.3 | 1049 | 0.02 | 42.7 |
| RL (from bad gains) | 9192 | 1.8 | 5369 | 0.35 | 9.9 |
| RL (from ZN gains) | 7889 | 3.6 | 1215 | 0.02 | 10.0 |

## cold_5V

| controller | IAE (°C·s) | overshoot % | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 11134 | 1.8 | 936 | 0.13 | 42.4 |
| Cohen-Coon | 10959 | 1.8 | 893 | 0.09 | 40.9 |
| SIMC | 10937 | 3.2 | 1205 | 0.03 | 42.5 |
| AMIGO | 11008 | 3.0 | 1281 | 0.03 | 44.0 |
| IMC | 13567 | 7.9 | 3043 | 0.02 | 40.2 |
| MATLAB pidtune | 14902 | 14.9 | 2575 | 0.00 | 0.8 |
| ZN re-tuned (oracle) | 11006 | 2.3 | 1109 | 0.03 | 43.7 |
| RL (from bad gains) | 14632 | 2.1 | 3902 | 0.01 | 19.9 |
| RL (from ZN gains) | 11292 | 3.8 | 1905 | 0.01 | 20.7 |

## supply_drop_5to3V

| controller | IAE (°C·s) | max deviation °C | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 601 | 0.63 | 330 | 0.08 | 48.9 |
| Cohen-Coon | 668 | 0.61 | 461 | 0.06 | 49.0 |
| SIMC | 1463 | 1.35 | 1416 | 0.03 | 48.7 |
| AMIGO | 1375 | 1.31 | 1188 | 0.01 | 48.7 |
| IMC | 5107 | 2.45 | 3194 | 0.18 | 40.2 |
| MATLAB pidtune | 5494 | 3.07 | 2729 | 0.12 | 0.9 |
| ZN re-tuned (oracle) | 593 | 0.74 | 330 | 0.04 | 48.6 |
| RL (from bad gains) | 1419 | 0.78 | 478 | 0.34 | 16.6 |
| RL (from ZN gains) | 1411 | 0.78 | 510 | 0.26 | 18.3 |

## supply_rise_3to5V

| controller | IAE (°C·s) | max deviation °C | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 1393 | 1.27 | 1551 | 0.07 | 47.6 |
| Cohen-Coon | 1090 | 1.24 | 612 | 0.08 | 47.4 |
| SIMC | 1802 | 2.14 | 2613 | 0.16 | 47.7 |
| AMIGO | 1991 | 2.11 | 3797 | 0.03 | 48.4 |
| IMC | 4954 | 3.47 | 2532 | 0.05 | 38.7 |
| MATLAB pidtune | 5391 | 4.96 | 1839 | 0.00 | 0.9 |
| ZN re-tuned (oracle) | 1457 | 1.04 | 4318 | 0.25 | 48.5 |
| RL (from bad gains) | 511 | 1.42 | 472 | 0.01 | 6.0 |
| RL (from ZN gains) | 537 | 1.49 | 483 | 0.01 | 6.7 |

## sp_step_4V

| controller | IAE (°C·s) | overshoot % | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 6295 | 2.0 | 1722 | 0.05 | 44.4 |
| Cohen-Coon | 5601 | 1.3 | 1590 | 0.03 | 43.7 |
| SIMC | 5177 | 3.2 | 1420 | 0.06 | 42.2 |
| AMIGO | 5874 | 2.1 | 1600 | 0.07 | 44.0 |
| IMC | 5186 | 3.5 | 1010 | 0.04 | 36.0 |
| MATLAB pidtune | 5995 | 7.2 | 2418 | 0.02 | 0.9 |
| ZN re-tuned (oracle) | 6295 | 2.0 | 1722 | 0.05 | 44.4 |
| RL (from bad gains) | 4421 | 1.4 | 1120 | 0.02 | 10.1 |
| RL (from ZN gains) | 4373 | 4.6 | 919 | 0.02 | 13.2 |

## Robustness: 40 random plants (K0, β, τ, θ, R, ambient, noise) and random scenarios

| controller | mean IAE (°C·s) | median IAE | runs not settled at the end | mean PWM chatter %/s |
|---|---|---|---|---|
| ZN | 8581 | 7090 | 12/40 | 42.9 |
| Cohen-Coon | 7735 | 6647 | 14/40 | 41.2 |
| SIMC | 6996 | 6023 | 10/40 | 39.2 |
| AMIGO | 8143 | 7013 | 12/40 | 41.6 |
| IMC | 8562 | 6940 | 17/40 | 36.4 |
| MATLAB pidtune | 9849 | 8765 | 12/40 | 0.9 |
| RL (from bad gains) | 5945 | 4827 | 2/40 | 10.8 |
| RL (from ZN gains) | 5706 | 4422 | 5/40 | 11.4 |
