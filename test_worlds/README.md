# test worlds

random arenas we tested the robot on (eval arena is different from the dev
one, so we needed these). open any `worlds/*.wbt` in webots R2025a and press play.

- `seedNN.wbt` - random cubes, random maze + tiles, robot starts in the S1 start
  zone with a random heading
- `seedNN_s2.wbt` - same arena as seedNN but robot starts in the chamber (like
  the S2-only attempts in rulebook 4.6)
- `dev_s2.wbt` / `dev_s2c1r1.wbt` - the official dev arena, robot starts in the
  chamber / in c1r1

`truth.json` has the real tile count for each world so you can check the LED.

each layout has its own `Arena_seedNN.proto` / `Obstacles_seedNN.proto` (made
by `tools/harness/gen.py`, the arena outer walls/gap/chamber are the same as
the real one, only cubes + maze + tiles change).

`controllers/main_controller/main_controller.py` here is a COPY of the root one
from when these were made. if you change the real controller, copy it over (or
re-run `python tools/experiments/build_testworlds.py`).

results (headless, judge supervisor, fast mode): every world right majority,
end cell reached, 0 collisions, ~73-92s for a full run.
