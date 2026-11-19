# test – Final Round test harness (docs/TEST_PLAN.md)

None of this is needed to run the robot. It tests the robot code on many random but legal arenas, times every run with a neutral judge, and compares it with the best possible run.

**Results so far:** see `PHASE_C_FINDINGS.md` and `results/phaseC.md`.

## Folders

- **`worlds/` + `protos/`:** the test worlds.
  - Open any `worlds/*.wbt` in Webots R2025a and press play.
  - They use the real `../simulation/protos/GuardianBot.proto`.
  - They also use the real robot code, through `controllers/test_runner`, so there is only one copy of it.
- **World names:**
  - `wNN`: full mission from START;
  - `wNN_maze`: the robot starts at the maze entrance;
  - `wNN_s3`: Section 3 restart with the saved clues;
  - `_hard`: 43 cubes, deceptive maze, more loops, 5–7 tiles;
  - `_deceptive`: dead ends near the exit, long side branches;
  - `_straight`: same arena as without it, robot placed exactly straight;
  - `w00_official`: the rulebook arena.
- **`truth/<world>.json`:** the true map:
  - cubes, maze walls, dead ends and tiles;
  - the clues;
  - the board and the robot's clan;
  - the correct winning cell(s).
- **`results/`:**
  - `runs.zip`: the raw data of every run (unzip it into `results/` to get `runs/`; `analyze.py` needs it);
  - `runs/<world>_<python|sil>/`: the judge's true path (`judge_path.csv`), the result, and the robot's log;
  - `ref/<world>.json`: the best possible run per section;
  - `analysis/<run>.json`: the numbers behind the table;
  - `plots/<run>.png`: real path over the best path, with stops; `overview_N.png` puts the plots side by side;
  - `phaseC.md` / `phaseC.csv`: the summary table.

## Tools (`tools/`, Linux, Webots R2025a)

| Command | What it does |
|---|---|
| `python tools/gen.py <name> <seed> [full\|maze\|s3]` | Make a test world. Env options: `NOBS=a-b`, `LOOPS=a-b`, `NTILES=a-b`, `DECEPTIVE=1`, `STRAIGHT=1`. Seed 0 = the official arena |
| `bash tools/run.sh <name> [python\|sil]` | One headless judged run. `sil` runs the real ESP32-S3 brain firmware via `simulation/controllers/mega_sil` |
| `python tools/reference.py <name>` | Best possible run per section on the true map, with GuardianBot's limits (0.30 m/s, 3.8 m/s², turn on the spot ≤ 5.2 rad/s) |
| `python tools/analyze.py --all` | Gap analysis, pictures and the table for every run |
| `python tools/contact.py "w*_python.png" 5` | Overview sheets of the pictures |

`tools/judge.py` is the supervisor that `run.sh` adds to its throwaway copy of the world. It is never in the test worlds themselves.
