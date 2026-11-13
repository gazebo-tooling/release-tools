import sys
from pathlib import Path

# pixi_ci lives in jenkins-scripts/tools
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
