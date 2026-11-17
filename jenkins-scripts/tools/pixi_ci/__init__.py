"""Build and test one Gazebo library with pixi and colcon.

See README.md in this directory for the job variables it reads.
"""

import sys
from pathlib import Path

# jenkins-scripts/dsl/tools holds scripts, not a package
sys.path.append(str(Path(__file__).resolve().parents[2] / "dsl" / "tools"))
