# GuardianBot v3.3 files (Mega + ESP32-S3 + ESP32-CAM, Mega shield PCB)

These are the v3.3 files that v4.0 replaced. They are kept here for reference and comparison.
The complete v3.3 tree, which runs as it was, is git tag `v3.3-pcb`. Its numbers are in `spec.yaml` in this folder.

| Folder / file | What it was |
|---|---|
| `firmware/esp32s3_brain/` | ESP32-S3 brain firmware (v4: `firmware/esp32cam_brain/`) |
| `firmware/esp32cam_vision/` | ESP32-CAM camera-only firmware (v4: the camera task in `esp32cam_brain`) |
| `cad/shield_v3/` | Mega shield PCB generator (paths assume its old place in `hardware/cad/`) |
| `pcb_v3/` | Mega shield KiCad project, Gerbers, toner PDFs, BOM, DRC |
| `GuardianBot_v3_build_guide.md` | v3 build guide and Rs 15,000 shopping list |
| `acrylic/`, `3d_prints/` | acrylic mid deck, Sharp IR mounts, 32 mm deck spacers |
