import sys
from pathlib import Path

# The scripts under test live in jenkins-scripts/dsl/tools
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
