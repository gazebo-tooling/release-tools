#!/usr/bin/env python3
"""Run the pixi_ci driver: python -u run_pixi_ci.py <library>

Puts this directory on sys.path instead of setting PYTHONPATH, so nothing
from the driver's import path leaks into the builds and tests it runs.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pixi_ci.__main__ import main  # noqa: E402

# Jenkins pipes stdout: on Windows python writes it in the ANSI code page,
# where text from tool reports may not fit. Replace it, as cmd's type does.
sys.stdout.reconfigure(errors="replace")
sys.exit(main())
