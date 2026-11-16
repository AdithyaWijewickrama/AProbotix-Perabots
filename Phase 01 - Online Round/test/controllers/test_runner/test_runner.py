# used by the test worlds in test/worlds. it just runs the real controller
# (controllers/main_controller/main_controller.py of this project) so there is
# only one copy of the robot code in the project.
import os
import runpy
import sys

here = os.path.dirname(os.path.abspath(__file__))
real = os.path.normpath(os.path.join(here, '..', '..', '..', 'controllers', 'main_controller', 'main_controller.py'))
sys.argv[0] = real
runpy.run_path(real, run_name='__main__')
