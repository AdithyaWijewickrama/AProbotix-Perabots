# AProbotix-Perabots

This repo holds team AProbotix's work for the PeraBots 2026 competition
(University category), Faculty of Engineering, University of Ruhuna.

| Folder | Round | What it is |
|---|---|---|
| [`Phase 01 - Online Round/`](Phase%2001%20-%20Online%20Round/) | Online (simulation) round | Webots solution submitted for round 1. Includes the TwoWheelRobot controller, test worlds, the sim lab and the submission zip builder |
| [`Phase 02 - Final Round/`](Phase%2002%20-%20Final%20Round/) | Final (physical) round – "The Nine Seals of Pera" | GuardianBot: Webots simulation of all 3 trials, plus the hardware (PCB, laser-cut acrylic, 3-D prints, parts list and build guide) |

## Phase 01 – Online Round

This is the original first-round project, moved into its own folder unchanged. Paths below
are relative to `Phase 01 - Online Round/`:

- `controllers/`, `protos/`, `worlds/`: working copy with comments. Open `worlds/Arena.wbt` in Webots R2025a.
- `submission/`: clean copy built by `make_submission_zip.py` (do not edit by hand).
- `test/`: test worlds, test report, tools and the sim lab (`test/tools/lab/start_lab.bat`).
- `CONTROLLER_EXPLANATION.md`, `DEVELOPMENT_NOTES.md`: write-ups of the controller.

## Phase 02 – Final Round

Start with [`Phase 02 - Final Round/README.md`](Phase%2002%20-%20Final%20Round/README.md).
Quick links:

- Simulation: open `Phase 02 - Final Round/simulation/worlds/perabots_arena.wbt` in Webots R2025a.
- Build guide and parts list: `Phase 02 - Final Round/docs/GuardianBot_build_guide_and_BOM.md`.
- PCB to order: `Phase 02 - Final Round/hardware/pcb/guardianbot_main_gerbers_JLCPCB.zip`.
- Laser cutting: `Phase 02 - Final Round/hardware/acrylic/`. 3-D printing: `Phase 02 - Final Round/hardware/3d_prints/`.
