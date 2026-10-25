# AProbotix-Perabots
This repo is for PeraBots Competition 2026 first round by AProbotix team in Faculty of Engineering University of Ruhuna

## whats where

- `submission/` - **the thing we submit.** no comments, only the files the
  rulebook (4.5) asks for. zip it as `AProbotix_Submission.zip`
- root `controllers/`, `protos/`, `worlds/` - our working copy, same code but
  with all the comments. open `worlds/Arena.wbt` in webots R2025a
- `CONTROLLER_EXPLANATION.md` - proper write up of what the controller does
- `DEVELOPMENT_NOTES.md` - how it works, what we learned, tuning knobs
- `test_worlds/` - 34 random arenas we tested on
- `tools/lab/` - **sim lab web app**: double click `start_lab.bat`, pick/generate
  worlds, change settings, run, compare (results kept in sqlite)
- `tools/` - headless test harness + the physics experiments

if you change the controller: change the root one, test it, then copy it to
`submission/` with the comments stripped.

note: the two drive wheels are moved 2cm forward (x 0 -> 0.02) in
TwoWheelRobot.proto. rulebook 4.4 allows small wheel position changes for
balance. everything else in the locked part is the original.
