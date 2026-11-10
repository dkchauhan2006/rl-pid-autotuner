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
| ZN | 2561 | 2.4 | 900 | 0.02 | 48.1 |
| Cohen-Coon | 2515 | 3.1 | 886 | 0.01 | 45.9 |
| SIMC | 2800 | 9.8 | 1506 | 0.00 | 25.3 |
| AMIGO | 2755 | 8.7 | 885 | 0.01 | 32.2 |
| IMC | 2792 | 10.4 | 1511 | 0.00 | 12.6 |
| MATLAB pidtune | 2791 | 10.5 | 1506 | 0.00 | 6.0 |
| ZN re-tuned (oracle) | 2472 | 0.7 | 886 | 0.02 | 51.9 |
| RL (from bad gains) | 2673 | 4.9 | 879 | 0.06 | 15.6 |
| RL (from ZN gains) | 2675 | 4.9 | 879 | 0.06 | 15.5 |

## cold_3V

| controller | IAE (°C·s) | overshoot % | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 5013 | 1.9 | 903 | 0.04 | 48.9 |
| Cohen-Coon | 4983 | 2.1 | 896 | 0.03 | 47.1 |
| SIMC | 5215 | 6.4 | 1396 | 0.00 | 25.9 |
| AMIGO | 5190 | 6.1 | 1371 | 0.01 | 32.9 |
| IMC | 5771 | 11.1 | 1809 | 0.00 | 12.3 |
| MATLAB pidtune | 5759 | 11.2 | 1786 | 0.00 | 2.7 |
| ZN re-tuned (oracle) | 4963 | 1.5 | 895 | 0.03 | 49.0 |
| RL (from bad gains) | 5044 | 5.1 | 1140 | 0.01 | 23.1 |
| RL (from ZN gains) | 5040 | 5.0 | 1133 | 0.01 | 23.0 |

## cold_4V

| controller | IAE (°C·s) | overshoot % | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 7789 | 2.8 | 968 | 0.02 | 42.6 |
| Cohen-Coon | 7787 | 2.8 | 962 | 0.02 | 42.8 |
| SIMC | 7995 | 4.8 | 1240 | 0.01 | 26.7 |
| AMIGO | 7949 | 4.6 | 1224 | 0.01 | 33.9 |
| IMC | 9109 | 10.7 | 1909 | 0.00 | 12.4 |
| MATLAB pidtune | 9258 | 12.2 | 1856 | 0.00 | 1.5 |
| ZN re-tuned (oracle) | 7789 | 2.8 | 968 | 0.02 | 42.6 |
| RL (from bad gains) | 7947 | 4.0 | 1001 | 0.00 | 3.7 |
| RL (from ZN gains) | 7858 | 4.0 | 1001 | 0.00 | 6.9 |

## cold_5V

| controller | IAE (°C·s) | overshoot % | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 10784 | 3.3 | 1032 | 0.05 | 34.1 |
| Cohen-Coon | 11138 | 3.3 | 1036 | 0.13 | 32.6 |
| SIMC | 10818 | 4.0 | 1114 | 0.01 | 27.4 |
| AMIGO | 10763 | 3.8 | 1093 | 0.02 | 33.6 |
| IMC | 12709 | 9.1 | 2142 | 0.00 | 12.6 |
| MATLAB pidtune | 13020 | 13.2 | 1861 | 0.00 | 1.0 |
| ZN re-tuned (oracle) | 10632 | 3.3 | 934 | 0.02 | 34.0 |
| RL (from bad gains) | 11148 | 5.2 | 1379 | 0.00 | 4.8 |
| RL (from ZN gains) | 11136 | 5.4 | 1361 | 0.00 | 4.8 |

## supply_drop_5to3V

| controller | IAE (°C·s) | max deviation °C | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 211 | 0.34 | 0 | 0.02 | 43.9 |
| Cohen-Coon | 203 | 0.34 | 0 | 0.02 | 43.8 |
| SIMC | 288 | 0.53 | 237 | 0.02 | 41.5 |
| AMIGO | 391 | 0.59 | 257 | 0.04 | 43.7 |
| IMC | 952 | 0.99 | 954 | 0.01 | 30.6 |
| MATLAB pidtune | 1008 | 1.39 | 892 | 0.00 | 2.9 |
| ZN re-tuned (oracle) | 245 | 0.41 | 0 | 0.02 | 43.6 |
| RL (from bad gains) | 222 | 0.79 | 271 | 0.01 | 24.3 |
| RL (from ZN gains) | 179 | 0.70 | 241 | 0.01 | 25.8 |

## supply_rise_3to5V

| controller | IAE (°C·s) | max deviation °C | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 520 | 1.76 | 283 | 0.05 | 35.1 |
| Cohen-Coon | 375 | 1.64 | 253 | 0.03 | 31.9 |
| SIMC | 1124 | 2.35 | 725 | 0.00 | 14.4 |
| AMIGO | 1084 | 2.49 | 621 | 0.00 | 18.7 |
| IMC | 2819 | 3.81 | 1101 | 0.00 | 6.9 |
| MATLAB pidtune | 2783 | 3.86 | 1087 | 0.00 | 1.5 |
| ZN re-tuned (oracle) | 355 | 1.56 | 240 | 0.03 | 35.6 |
| RL (from bad gains) | 431 | 1.42 | 413 | 0.00 | 6.8 |
| RL (from ZN gains) | 404 | 1.35 | 413 | 0.00 | 10.3 |

## sp_step_4V

| controller | IAE (°C·s) | overshoot % | settling s | ss error °C | PWM chatter %/s |
|---|---|---|---|---|---|
| ZN | 4300 | 1.8 | 774 | 0.04 | 37.2 |
| Cohen-Coon | 4298 | 1.9 | 774 | 0.04 | 37.0 |
| SIMC | 4393 | 4.8 | 1026 | 0.01 | 25.2 |
| AMIGO | 4392 | 4.1 | 779 | 0.02 | 30.8 |
| IMC | 5088 | 11.5 | 1726 | 0.00 | 12.6 |
| MATLAB pidtune | 5176 | 13.2 | 1693 | 0.00 | 1.6 |
| ZN re-tuned (oracle) | 4300 | 1.8 | 774 | 0.04 | 37.2 |
| RL (from bad gains) | 4322 | 5.8 | 1054 | 0.00 | 8.1 |
| RL (from ZN gains) | 4322 | 5.8 | 1054 | 0.00 | 8.7 |

## Robustness: 40 random plants (K0, β, τ, θ, R, ambient, noise) and random scenarios

| controller | mean IAE (°C·s) | median IAE | runs not settled at the end | mean PWM chatter %/s |
|---|---|---|---|---|
| ZN | 6206 | 4521 | 3/40 | 37.1 |
| Cohen-Coon | 6032 | 4317 | 3/40 | 36.1 |
| SIMC | 6162 | 4740 | 1/40 | 31.3 |
| AMIGO | 6518 | 4874 | 2/40 | 34.6 |
| IMC | 6969 | 5349 | 2/40 | 22.3 |
| MATLAB pidtune | 7085 | 5586 | 0/40 | 4.3 |
| RL (from bad gains) | 5673 | 3942 | 1/40 | 14.3 |
| RL (from ZN gains) | 5723 | 3890 | 0/40 | 14.0 |
