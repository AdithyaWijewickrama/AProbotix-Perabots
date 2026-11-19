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
| v3.3 + Mega shield PCB | `v3.3-pcb` | all wiring on a local-made PCB | 152.1 s | Rs 13,750 |
| v4.0 ESP32-CAM brain + PCB deck | `v4.0` / `v4.0.1` | real inventory: 8 ToF, QTR-8RC, DRV8833 + 2S, PCB middle floor | 153.6 s | Rs 20,000 (vero: 15,500) |
| **v4.1 ESP32-S3 + OV7670 brain** | `v4.1` | the team's own S3 + OV7670 instead of a bought ESP32-CAM; S3 flat on the top deck; Wi-Fi test-bench page | 153.6 s (firmware: 175.6 s) | Rs 15,900 (vero: 11,400) |

Times come from one neutral "judge" supervisor on the same arena for every version
(`judge/judge.py`).

**Website:** the GitHub Pages site that `.github/workflows/versions-site.yml` rebuilds on every push
(or run `python build_site.py` and open `_site/index.html` through a local server, e.g. `python -m http.server`
inside `_site`). It has two pages:

- `index.html`: version cards (with the tests done on each version), charts and a side-by-side comparison.
- `viewer.html`: **hardware in 3-D** for v2.0, v3.3, v4.0 and v4.1. You can turn and zoom the PCB with every part (and
  the Arduino Mega / ESP32 board where it sits), every 3-D printed part, and every laser-cut deck in 3-D or as its
  cut drawing. It also has the downloads: Gerbers, schematic, BOM, DXF, STL and the build guide.

The 3-D files come from `export_models.py`. Run it after a board or CAD file changes; it needs KiCad 7 and
`pip install trimesh mapbox_earcut shapely pillow ezdxf matplotlib`. It writes `<version>/models/` (`pcb.glb`,
the deck drawings and `manifest.json`). The PCB model is built from the board file itself: real copper,
solder-mask openings, silkscreen and holes on both sides, plus a simple body for every part made from its
footprint. three.js is included in `viewer_lib/` (MIT licence), so the page needs no CDN.

**Add a version:**

1. Create `vX.Y_name/spec.yaml` (copy the latest one) and `images/iso.jpg`.
   List the tests run on that version under `tests:` (name, result, status pass/partial/fail/info, optional
   repo-relative `link`). Leave it as `tests: []` if none were run; the site then shows "-".
2. If it has hardware, add it to `VERSIONS` in `export_models.py` and run that script.
3. Push.
Run `python build_site.py` to preview locally (`pip install pyyaml`).
