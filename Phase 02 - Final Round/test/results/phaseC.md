# Phase C - real runs vs the best possible run

| run | start | result | S1 real/best s | S2 real/best s | S3 real/best s | END | tile count | tiles found | piece on | collisions | contact flags | min clearance mm |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| w00_official_python | full | mission complete, piece correct | 24.1 / 11.0 | 37.5 / 37.5 | 91.9 / 21.9 | 153.6 s | right | 5/5 | 7 (right) | 0 | 0 | 17.6 |
| w00_official_sil | full | mission complete, piece correct | 35.5 / 11.0 | 46.5 / 37.5 | 94.0 / 21.9 | 176.0 s | right | 5/5 | 7 (right) | 0 | 0 | 10.1 |
| w01_python | full | mission complete, piece correct | 28.7 / 12.5 | 43.3 / 37.9 | 89.5 / 21.4 | 161.6 s | right | 3/3 | 6 (right) | 0 | 0 | 12.0 |
| w01_sil | full | DID NOT FINISH (no movement for 60 s), piece not on a correct cell | 96.3 / 12.5 | - | - | - | - | - | not placed | 0 | 0 | 5.3 |
| w02_python | full | mission complete, piece correct | 15.7 / 11.2 | 47.6 / 38.5 | 87.0 / 21.4 | 150.3 s | right | 3/3 | 8 (right) | 0 | 0 | 31.3 |
| w02_sil | full | DID NOT FINISH (no movement for 60 s), piece not on a correct cell | 84.9 / 11.2 | - | - | - | - | - | not placed | 0 | 0 | 33.5 |
| w02_straight_python | full | mission complete, piece correct | 14.5 / 11.2 | 47.7 / 38.5 | 87.0 / 21.4 | 149.2 s | right | 3/3 | 8 (right) | 0 | 0 | 22.3 |
| w02_straight_sil | full | mission complete, piece correct | 22.5 / 11.2 | 55.4 / 38.5 | 88.3 / 21.4 | 166.1 s | right | 3/3 | 8 (right) | 0 | 0 | 20.1 |
| w03_deceptive_python | full | mission complete, piece correct | 16.8 / 11.8 | 43.0 / 37.9 | 81.5 / 18.1 | 141.3 s | right | 3/3 | 5 (right) | 0 | 4 | 27.3 |
| w03_deceptive_sil | full | mission complete, piece correct | 36.0 / 11.8 | 60.5 / 37.9 | 82.5 / 18.1 | 179.1 s | right | 3/3 | 5 (right) | 0 | 1 | 18.1 |
| w04_python | full | mission complete, piece correct | 14.2 / 10.2 | 43.9 / 39.5 | 90.1 / 21.4 | 148.2 s | right | 3/3 | 6 (right) | 0 | 0 | 28.5 |
| w05_loops_python | full | mission complete, piece correct | 15.9 / 10.8 | 51.8 / 40.7 | 90.1 / 21.9 | 157.8 s | right | 3/3 | 4 (right) | 0 | 0 | 17.8 |
| w06_maze_python | maze | mission complete, piece correct | - | 42.4 / 29.1 | 83.9 / 21.9 | 126.3 s | right | 5/5 | 7 (right) | 0 | 0 | 33.2 |
| w06_maze_sil | maze | mission complete, piece correct | - | 52.1 / 29.1 | 85.8 / 21.9 | 138.0 s | right | 5/5 | 7 (right) | 0 | 2 | 26.9 |
| w07_maze_deceptive_python | maze | mission complete, piece correct | - | 48.4 / 43.1 | 91.7 / 21.9 | 140.0 s | right | 3/3 | 7 (right) | 0 | 3 | 32.1 |
| w08_s3_python | s3 | mission complete, piece correct | - | - | 43.0 / 21.9 | 46.2 s | - | - | 7 (right) | 0 | 0 | 34.0 |
| w09_s3_python | s3 | mission complete, piece correct | - | - | 97.7 / 21.9 | 100.9 s | - | - | 7 (right) | 0 | 0 | 34.0 |
| w10_python | full | mission complete, piece correct | 20.7 / 11.2 | 46.4 / 22.2 | 81.4 / 18.1 | 148.5 s | right | 3/3 | 5 (right) | 0 | 0 | 23.9 |
| w11_python | full | mission complete, piece correct | 20.3 / 11.0 | 52.3 / 45.8 | 81.5 / 18.1 | 154.1 s | right | 3/3 | 5 (right) | 0 | 0 | 31.6 |
| w12_python | full | mission complete, piece correct | 17.5 / 10.5 | 55.5 / 49.2 | 81.0 / 18.1 | 154.1 s | right | 3/3 | 5 (right) | 0 | 1 | 25.7 |
| w13_python | full | mission complete, piece correct | 20.2 / 12.8 | 46.9 / 39.8 | 42.2 / 21.4 | 109.4 s | right | 3/3 | 8 (right) | 0 | 0 | 35.1 |
| w14_python | full | mission complete, piece correct | 21.5 / 10.7 | 52.6 / 29.3 | 42.0 / 21.4 | 116.1 s | right | 5/5 | 8 (right) | 0 | 0 | 23.8 |
| w15_python | full | DID NOT FINISH (no movement for 60 s), piece not on a correct cell | 71.6 / 9.7 | - | - | - | - | - | not placed | 0 | 0 | 41.0 |
| w16_maze_python | maze | mission complete, piece correct | - | 49.8 / 46.6 | 81.2 / 18.1 | 131.0 s | right | 3/3 | 5 (right) | 0 | 0 | 28.4 |
| w17_maze_python | maze | mission complete, piece correct | - | 51.8 / 37.1 | 81.5 / 18.1 | 133.3 s | right | 3/3 | 5 (right) | 0 | 5 | 33.9 |
| w18_maze_python | maze | mission complete, piece correct | - | 59.6 / 51.6 | 109.5 / 21.9 | 169.1 s | right | 3/3 | 4 (right) | 0 | 0 | 22.8 |
| w19_maze_python | maze | mission complete, piece correct | - | 36.9 / 34.5 | 87.4 / 21.4 | 124.3 s | right | 3/3 | 8 (right) | 0 | 3 | 36.2 |
| w20_maze_python | maze | mission complete, piece correct | - | 56.8 / 43.3 | 75.8 / 18.1 | 132.7 s | right | 3/3 | 9 (right) | 0 | 0 | 20.6 |
| w21_maze_python | maze | mission complete, piece correct | - | 45.7 / 39.0 | 89.7 / 21.4 | 135.4 s | right | 3/3 | 6 (right) | 0 | 0 | 32.6 |
| w22_s3_python | s3 | mission complete, piece correct | - | - | 89.6 / 21.4 | 92.9 s | - | - | 6 (right) | 0 | 0 | 34.0 |
| w23_s3_python | s3 | mission complete, piece correct | - | - | 75.8 / 18.1 | 79.0 s | - | - | 9 (right) | 0 | 0 | 34.0 |
| w24_s3_python | s3 | mission complete, piece correct | - | - | 83.5 / 21.9 | 86.7 s | - | - | 7 (right) | 0 | 0 | 34.0 |
| w25_s3_python | s3 | mission complete, piece correct | - | - | 81.2 / 18.1 | 84.6 s | - | - | 5 (right) | 0 | 0 | 34.0 |
| w26_hard_python | full | mission complete, piece correct | 26.1 / 12.3 | 31.1 / 29.4 | 101.4 / 21.9 | 158.6 s | right | 5/5 | 3 (right) | 0 | 3 | 16.1 |
| w26_hard_sil | full | DID NOT FINISH (no movement for 60 s), piece not on a correct cell | 99.0 / 12.3 | - | - | - | - | - | not placed | 0 | 0 | 27.8 |
| w26_hard_straight_python | full | mission complete, piece correct | 28.0 / 12.3 | 29.8 / 29.4 | 101.7 / 21.9 | 159.6 s | right | 5/5 | 3 (right) | 0 | 2 | 16.6 |
| w26_hard_straight_sil | full | WRONG TILE COUNT, piece not on a correct cell | 46.1 / 12.3 | 36.0 / 29.4 | 110.5 / 21.9 | 192.6 s | WRONG | 5/5 +1 false | 1 (WRONG) | 0 | 7 | 26.6 |
| w27_hard_python | full | WRONG TILE COUNT | 16.5 / 9.8 | 30.8 / 28.3 | 99.1 / 21.9 | 146.4 s | WRONG | 4/5 | 7 (right) | 0 | 0 | 36.2 |
| w28_hard_python | full | mission complete, piece correct | 15.1 / 11.3 | 33.7 / 30.9 | 40.1 / 18.7 | 88.9 s | right | 5/5 | 1 (right) | 0 | 0 | 22.7 |
| w29_hard_python | full | mission complete, piece correct | 21.9 / 12.7 | 47.5 / 28.6 | 87.1 / 21.4 | 156.5 s | right | 5/5 | 8 (right) | 0 | 0 | 16.6 |

## Where the extra time goes (average seconds per section, finished runs only)

| section | runs | avg gap s | longer path | turning | stops | below top speed | other |
|---|---|---|---|---|---|---|---|
| S1 | 21 | 11.3 | 1.9 | 6.3 | 2.5 | 0.6 | 0.1 |
| S2 | 30 | 9.5 | 2.6 | 3.7 | 2.5 | 0.7 | 0.1 |
| S3 | 36 | 62.9 | 15.6 | 5.2 | 2.9 | 39.2 | 0.1 |
