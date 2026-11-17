# GuardianBot versions

The **latest version always lives in the top-level folders** of `Phase 02 - Final Round`
(`firmware/`, `hardware/`, `simulation/`, `docs/`). This folder keeps every earlier version, plus
one `spec.yaml` per version, so bodies, PCBs, budgets, speed, weight and parts can be compared.

| Version | Git tag | What changed | END reached (sim) | Budget |
|---|---|---|---|---|
| v1.0 TwoWheelRobot | `v1.0-twowheel` | first Webots robot | did not finish | - |
| v1.1 GuardianBot ToF | `v1.1-tof` | 130 mm round body, rear pocket, 10 ToF | did not finish (stopped in S3) | ≈ USD 330 |
| v2.0 GuardianBot | `v2.0` | HC-SR04 + Sharp, custom round PCB | 180.7 s | ≈ USD 230 |
| v3.0 Mega Guardian (draft) | `v3.0-mega-draft` | Mega-only firmware, assumed parts | not simulated | Rs 10,500-14,000 |
| v3.1 Mega + ESP32-S3 | `v3.1` | the team's real parts, 3 decks | 182.9 s | Rs 5,700 |
| v3.2 + ESP32-CAM | `v3.2-camera` | camera reads dead ends from a distance | 152.1 s | Rs 10,050 |
| **v3.3 + Mega shield PCB** | `v3.3-pcb` | all wiring on a local-made PCB | 152.1 s | Rs 13,750 |

Times come from one neutral "judge" supervisor on the same arena for every version
(`judge/judge.py`).

**Website:** `_site/index.html` (open it locally), or the GitHub Pages site that
`.github/workflows/versions-site.yml` rebuilds on every push.

**Add a version:** create `vX.Y_name/spec.yaml` (copy the latest one) and `images/iso.jpg`, then push.
Run `python build_site.py` to preview locally (`pip install pyyaml`).
