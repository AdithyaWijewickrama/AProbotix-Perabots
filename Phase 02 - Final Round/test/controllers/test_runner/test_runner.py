"""Test worlds use this controller. It runs the real robot code (simulation/controllers/main_controller),
so there is only one copy of it. For a Section 3 restart world it first writes the clues the robot would
have saved after the maze (controllerArgs "clues=blue,green,right,left")."""
import json
import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REAL = os.path.normpath(os.path.join(HERE, "..", "..", "..", "simulation", "controllers", "main_controller", "main_controller.py"))
for a in sys.argv[1:]:
    if a.startswith("clues="):
        json.dump([int(v) for v in a[6:].split(",")], open(os.path.join(os.path.dirname(REAL), "clues.json"), "w"))
sys.path.insert(0, os.path.dirname(REAL))
runpy.run_path(REAL, run_name="__main__")
