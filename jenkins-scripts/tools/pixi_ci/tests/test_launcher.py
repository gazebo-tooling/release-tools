import os
import subprocess
import sys
from pathlib import Path

LAUNCHER = Path(__file__).resolve().parents[2] / "run_pixi_ci.py"


def test_launcher_imports_the_driver_without_pythonpath(tmp_path):
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    result = subprocess.run([sys.executable, "-u", str(LAUNCHER), "--help"],
                            cwd=tmp_path, env=env, capture_output=True,
                            text=True)
    assert result.returncode == 0
    assert result.stdout.startswith("usage: pixi_ci")
