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


def test_launcher_output_does_not_fail_on_the_code_page(tmp_path):
    # Jenkins pipes stdout, so python on Windows writes it in the ANSI code
    # page, strictly: a dxdiag report with text outside it raised
    # UnicodeEncodeError where cmd's type printed it
    script = "\n".join([
        "import runpy, sys",
        f"sys.path.insert(0, {str(LAUNCHER.parent)!r})",
        "import pixi_ci.__main__ as driver",
        "driver.main = lambda: print('Card name: \\u4e2d NVIDIA') or 0",
        f"sys.argv = [{str(LAUNCHER)!r}]",
        f"runpy.run_path({str(LAUNCHER)!r}, run_name='__main__')"])
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    env["PYTHONIOENCODING"] = "ascii"
    result = subprocess.run([sys.executable, "-c", script], cwd=tmp_path,
                            env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout == "Card name: ? NVIDIA\n"
