# tools

not part of the submission. needs webots at `D:\Programs\Webots` (change the
paths in `harness/run.ps1` if yours is somewhere else) and python on PATH.

## lab/ - web app for testing (start here)

double click `lab/start_lab.bat` -> http://localhost:8765. Pick or generate
worlds, change any controller setting, run, and compare against older runs
(sqlite). see `lab/README.md`.

generator options (env vars for gen.py, also in the lab UI): `NCUBES=a-b`,
`NTILES=a-b`, `LOOPS=a-b`. Defaults give the same worlds as before.

## harness/ - headless test runs with a judge

- `run.ps1 <seed> [s2]` - builds a throwaway project in `harness/runs/`
  (random arena from the seed, seed 0 = dev arena), runs webots in fast mode
  with no rendering, prints the controller log + the judge result
- `batch.ps1 -seeds 1,2,3 [-mode s2]` - runs several and checks the LED
  colour against the truth
- `gen.py` - makes the arena/obstacles/world. env vars: `START=x,y,yaw`,
  `NOOBS=1` (no cubes), `CTRL=path` (test another controller file), `LAM=`,
  `EST=1` (log the pose every step, needed for ana/tb/route)
- `judge.py` - supervisor in the test world: true position, collisions,
  time to S1 / end cell, tilt. quits the sim when done
- `ana.py`, `tb.py`, `route.py` - log analysis (pose error vs truth, time in
  pivot/drive/idle, maze cell order)

windows note: webots doesnt pipe controller prints to the console, so the
harness makes the controllers write `ctrl_log.txt` / `judge_log.txt` instead.

## experiments/ - the physics tests that decided the design

- `calib.py` - pivots and straight runs, showed the robot pivots ~2.5cm off
  the axle and slips on curves
- `calib2.py` - heading drift vs heading angle, wheels first vs caster first.
  this is the one that showed caster first locks onto the axes
- `mkvar.py` - make a controller copy with other accel numbers (used to pick
  MAX_DV 4 / ACC 3.8)
- `build_testworlds.py` - rebuilds `test/worlds` + `test/protos` from the seeds

## report.py

`python test/tools/report.py` writes `test/TEST_REPORT.md` and
`test/results/*.csv` from the lab database
