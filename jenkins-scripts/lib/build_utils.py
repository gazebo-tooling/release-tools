"""Shared build utilities for the Gazebo CI pixi/conda build system.

Provides platform-independent helpers: command execution with logging,
Jenkins section markers, file download with retries, and CMake version
detection.
"""

import re
import subprocess
import sys
import time
import urllib.request
from contextlib import contextmanager


def run_cmd(cmd, cwd=None, env=None, check=True):
    """Run a command with logging and error handling.

    Prints the command being run, streams output to stdout/stderr,
    and raises CalledProcessError on failure when check=True.

    Args:
        cmd: Command as a list of strings.
        cwd: Working directory for the command.
        env: Environment variables dict. If None, inherits current env.
        check: If True, raise on non-zero exit code.

    Returns:
        subprocess.CompletedProcess instance.
    """
    print(f"+ {' '.join(cmd)}", flush=True)
    return subprocess.run(
        cmd,
        cwd=cwd,
        env=env,
        check=check,
    )


@contextmanager
def section(name):
    """Context manager that prints Jenkins log-folding section markers.

    Prints ``# BEGIN SECTION: {name}`` on entry and ``# END SECTION``
    on exit, which Jenkins uses to fold log output.
    """
    print(f"# BEGIN SECTION: {name}", flush=True)
    try:
        yield
    finally:
        print("# END SECTION", flush=True)


def download_file(url, dest_path, retries=3):
    """Download a file from *url* to *dest_path* with retry logic.

    Replaces the PowerShell ``Invoke-WebRequest`` call used in the
    batch :wget helper.

    Args:
        url: URL to download.
        dest_path: Local filesystem path to write the file to.
        retries: Number of attempts before giving up.

    Raises:
        RuntimeError: If all retry attempts are exhausted.
    """
    for attempt in range(1, retries + 1):
        try:
            print(f"Downloading {url} (attempt {attempt}/{retries})",
                  flush=True)
            urllib.request.urlretrieve(url, dest_path)
            return
        except Exception as exc:
            print(f"Download failed: {exc}", flush=True)
            if attempt >= retries:
                raise RuntimeError(
                    f"Failed to download {url} after {retries} attempts"
                ) from exc
            time.sleep(5)


def detect_cmake_major_version(cmakelists_path):
    """Extract the project major version from a CMakeLists.txt file.

    Tries several regex patterns used across the Gazebo ecosystem:
    PROJECT_MAJOR_VERSION, SDF_MAJOR_VERSION, ignition-* project names,
    and generic ``project(... VERSION N)`` declarations.

    Args:
        cmakelists_path: Path to the CMakeLists.txt file.

    Returns:
        The major version as a string (e.g. ``"8"``).

    Raises:
        SystemExit: If no version pattern is found.
    """
    with open(cmakelists_path, "r") as f:
        txt = f.read()

    patterns = [
        r'set *\( *PROJECT_MAJOR_VERSION +(\d+)',
        r'set *\( *SDF_MAJOR_VERSION +(\d+)',
        r'project *\( *ignition-[a-z\-_]+(\d+)',
        r'project *\(.*VERSION +(\d+)',
    ]

    for pattern in patterns:
        match = re.search(pattern, txt)
        if match:
            return match.group(1)

    sys.exit("could not detect the major version")
