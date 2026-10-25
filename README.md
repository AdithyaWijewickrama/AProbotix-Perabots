# AProbotix-Perabots
This repo is for PeraBots Competition 2026 first round by AProbotix team in Faculty of Engineering University of Ruhuna

## whats where

- root `controllers/`, `protos/`, `worlds/` - our working copy WITH comments.
  open `worlds/Arena.wbt` in webots R2025a. **edit the robot here.**
- `submission/` - the clean copy (no comments). made automatically by
  `make_submission_zip.py`, dont edit it by hand
- `test/` - test worlds (45), test report, raw results, test tools
  (`test/README.md`)
  - `test/tools/lab/start_lab.bat` - **sim lab web app**: pick/generate worlds,
    change settings, run, compare (results kept in sqlite)
- `CONTROLLER_EXPLANATION.md` - proper write up of what the controller does
- `DEVELOPMENT_NOTES.md` - how it works, what we learned, tuning knobs

## making the submission zip

```
python make_submission_zip.py
```

this rebuilds `submission/` from the root files (strips comments and checks the
code didnt change), refreshes `test/TEST_REPORT.md` from the lab database, and
writes `dist/AProbotix_Submission.zip`. inside the zip:
`PeraBots_UC/{protos, controllers/main_controller, worlds, docs, test}`.
there is exactly one TwoWheelRobot.proto and one main_controller.py in it (the
script checks that), the test worlds reuse them through `test/controllers/test_runner`.

note: the two drive wheels are moved 2cm forward (x 0 -> 0.02) in
TwoWheelRobot.proto. rulebook 4.4 allows small wheel position changes for
balance. everything else in the locked part is the original.
