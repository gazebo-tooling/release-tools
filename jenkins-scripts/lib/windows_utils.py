"""Windows-specific build utilities.

Isolates all Windows-only operations (MSVC compiler setup, GPU detection,
OGRE paths) so that a future ``macos_utils.py`` can provide platform
equivalents without touching shared code.
"""

import os
import subprocess
import sys

from build_utils import run_cmd, section


# MSVC vcvarsall.bat search paths, in priority order.
_VCVARSALL_CANDIDATES = [
    # VS 2022 Community (32-bit Windows or native 64-bit install)
    r"C:\Program Files\Microsoft Visual Studio\2022\Community"
    r"\VC\Auxiliary\Build\vcvarsall.bat",
    # VS 2019 Enterprise (64-bit Windows, 32-bit Program Files)
    r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise"
    r"\VC\Auxiliary\Build\vcvarsall.bat",
    # VS 2019 Enterprise (32-bit Windows)
    r"C:\Program Files\Microsoft Visual Studio\2019\Enterprise"
    r"\VC\Auxiliary\Build\vcvarsall.bat",
    # VS 2019 Community (64-bit Windows, 32-bit Program Files)
    r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Community"
    r"\VC\Auxiliary\Build\vcvarsall.bat",
    # VS 2019 Community (32-bit Windows)
    r"C:\Program Files\Microsoft Visual Studio\2019\Community"
    r"\VC\Auxiliary\Build\vcvarsall.bat",
]


def configure_msvc_compiler(platform="x86_amd64"):
    """Find and invoke vcvarsall.bat to configure the MSVC compiler.

    Searches the standard Visual Studio installation paths and calls
    the first ``vcvarsall.bat`` found with the requested *platform*
    target.  Captures the resulting environment variables and injects
    them into the current process so that subsequent ``subprocess``
    calls inherit the compiler settings.

    Replaces ``:configure_msvc2019_compiler`` in ``windows_library.bat``.

    Args:
        platform: MSVC platform string.  Common values:
            ``"x86_amd64"`` (cross-compile 64-bit from 32-bit host,
            default), ``"x86"`` (32-bit).
    """
    with section("configure the MSVC compiler"):
        # Skip if already configured
        if os.environ.get("VSCMD_VER"):
            print("VS compiler already configured", flush=True)
            return

        # Strip stray quotes from PATH (Jenkins workaround)
        os.environ["PATH"] = os.environ.get("PATH", "").replace('"', '')

        msvc_keyword = platform
        if platform == "x86":
            print("Using 32bits VS configuration", flush=True)
        else:
            print("Using 64bits VS configuration", flush=True)
            msvc_keyword = "x86_amd64"
            os.environ["PLATFORM_TO_BUILD"] = "amd64"
            os.environ["PreferredToolArchitecture"] = "x64"

        vcvarsall = None
        for candidate in _VCVARSALL_CANDIDATES:
            if os.path.isfile(candidate):
                vcvarsall = candidate
                break

        if vcvarsall is None:
            sys.exit("Could not find vcvarsall.bat")

        # Call vcvarsall.bat and capture the resulting environment
        capture_cmd = f'call "{vcvarsall}" {msvc_keyword} && set'
        result = subprocess.run(
            ["cmd", "/c", capture_cmd],
            capture_output=True,
            text=True,
            check=True,
        )

        for line in result.stdout.splitlines():
            if "=" in line:
                key, _, value = line.partition("=")
                os.environ[key] = value


def check_gpu_support(workspace):
    """Run dxdiag and verify NVIDIA GPU presence.

    Replaces the GPU check in ``colcon-default-devel-windows.bat:48-60``.

    Args:
        workspace: Path to the Jenkins workspace (used to write
            ``dxdiag.txt``).

    Raises:
        SystemExit: If no NVIDIA GPU is detected.
    """
    with section("dxdiag info"):
        dxdiag_file = os.path.join(workspace, "dxdiag.txt")
        run_cmd(["dxdiag", "/t", dxdiag_file])

        with open(dxdiag_file, "r") as f:
            content = f.read()
        print(content, flush=True)

        print(f"Checking for correct NVIDIA GPU support {dxdiag_file}",
              flush=True)
        if "Manufacturer: NVIDIA" not in content:
            sys.exit("ERROR: NVIDIA GPU not found in dxdiag")


def get_ogre_env_vars(conda_prefix):
    """Return OGRE resource path environment variables.

    Uses Windows-style paths under the conda prefix.

    Replaces lines 123-124 of ``colcon-default-devel-windows.bat``.

    Args:
        conda_prefix: The ``CONDA_PREFIX`` path from the active
            pixi environment.

    Returns:
        A dict with ``OGRE_RESOURCE_PATH`` and ``OGRE2_RESOURCE_PATH``.
    """
    return {
        "OGRE_RESOURCE_PATH": os.path.join(conda_prefix, "Library", "bin"),
        "OGRE2_RESOURCE_PATH": os.path.join(
            conda_prefix, "Library", "bin", "OGRE-Next"
        ),
    }
