# test

everything we used to test the controller. none of this is needed to run the
robot, the evaluators only need `protos/` + `controllers/main_controller/`.

- `TEST_REPORT.md` - results of the final controller on all 45 test worlds,
  the settings we compared, and what we found
- `results/` - raw data (csv). `final_controller.csv` = the final code on every
  world, `all_runs.csv` = every run we did in the lab
- `worlds/` + `protos/` - the 45 test worlds. open any `worlds/*.wbt` in webots
  R2025a and press play
  - `seedNN` - random arena, robot in the S1 start zone, random heading
  - `seedNN_s2` - same arena, robot placed at the maze entrance (rule 4.6)
  - `seedNN_hard` - 30-40 cubes, 8-9 tile slots, more maze loops
  - `dev_s2`, `dev_s2c1r1` - the official dev arena with a chamber / c1r1 start
    (the dev arena with the normal start is `../worlds/Arena.wbt`)
- `truth.json` - real number of blue/green tiles in each test world
- `controllers/test_runner/` - the test worlds use this. it just runs the real
  `../controllers/main_controller/main_controller.py`, so there is only one copy
  of the robot code. the test worlds also use the real `../protos/TwoWheelRobot.proto`
- `tools/` - sim lab web app, headless harness + judge, physics experiments,
  report generator. see `tools/README.md`

the random arenas keep everything the rulebook fixes (outer walls, gap,
chamber, 9x4 maze, 25cm cells, 10cm cubes, 25cm passages). only cubes, maze
walls and tiles change. the protos are named `Arena_seedNN.proto` etc so they
cant be mixed up with the real `../protos/Arena.proto`.
