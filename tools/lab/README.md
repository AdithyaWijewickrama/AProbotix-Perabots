# sim lab

small local web app to test the controller on lots of worlds with different
settings, keep every result in sqlite and compare them.

**start:** double click `start_lab.bat` (or `python tools/lab/lab.py`) and open
http://localhost:8765. only needs python 3 (stdlib, nothing to install) and
webots R2025a. set the webots folder in the Settings tab if it isnt
`D:\Programs\Webots`.

## tabs

- **Run**
  1. pick worlds. Either tick the 35 test worlds (same as `test_worlds/`), or
     generate new ones from seeds. For generated worlds you can set the start
     (S1 zone / chamber / c1r1 / custom x,y,heading), the number of cubes,
     tiles and extra maze loops, or no cubes.
  2. controller. Use the root (commented) file, `submission/`, or any other .py.
     Every tunable number is listed with its current value. Change any of them;
     blank or unchanged = file value. A **sweep** queues one config per value
     (e.g. MAX_DV = 2,3,4,5).
  3. how to run. Headless fast (batches, ~20 s per run), window fast, or
     window real time x1 to watch. Set the sim time limit and a label (the
     config name used for comparing).
- **Results**: every run with result (PASS / WRONG COLOUR / NOT FINISHED /
  COLLISION / TILT / CRASH), tile counts vs truth, S1 time, decision time,
  total time, distance, collisions, max tilt, changed settings. Sort, filter,
  delete, relabel. Click a run for the logs. From there you can open its world
  in webots, export the exact controller it used (`tools/lab/exports/`), or
  run it again.
- **Compare**: pick labels. You get PASS counts, mean/best/worst for a metric
  on the worlds they all ran (fair comparison), % vs a baseline, a bar chart,
  which settings differ, and a per-world table (best = green, fail = red).
- **Recommendations**: computed from the database (best config vs
  "default" on shared worlds, failures, rocking, not enough worlds tested)
  plus the general tips we learned.

## where stuff is

- `lab.db`: sqlite, table `runs` (one row per run, all settings + results) and
  `settings`. Not in git. Delete it to start fresh.
- `runs/run_N/`: the generated webots project for run N (world, protos,
  controller copy, logs). Not in git.
- `exports/`: controllers exported from runs. Not in git.

the controller settings are changed by regex on a copy of the file, so the
real controller is never touched. if someone renames one of those constants
in the controller, the lab shows "(not found in this file)" for it instead
of silently doing nothing.

## our recommendations for using it

1. run `default` on all 35 test worlds first, so everything has a baseline
   to compare against.
2. change one thing at a time (or sweep one param), on the same worlds.
3. a config is only better if it is 100% PASS on 10+ S1-start worlds and
   5+ maze-entrance worlds **and** faster on average. Check the max tilt
   column too (over ~3 deg = bouncing).
4. stress test the winner on harder generated worlds (cubes 40-45, loops 0,
   tiles 7, new seeds like 100-120) before trusting it.
5. watch 1-2 runs in real time. Numbers dont show scraping or shaking.
6. then put the numbers in the root controller, copy it to `submission/`
   without comments and rebuild the zip.

things we already know from testing: V_MAX > 0.2 does nothing (motor cap).
MAX_DV ~3-4 is the sweet spot (10 rocks the robot and is slower). Keep W_RUN
tiny (steering while moving = wheel slip). LAM 1 beat 0 and 0.5.
R_HARD < 0.06 risks the body swinging into cubes when pivoting.
