"""Pixi/conda environment management utilities.

Handles pixi binary installation, environment creation, shell activation,
and conda environment detection.  Platform-aware via ``sys.platform``
checks so that adding macOS support later requires only extending the
URL / binary-name helpers.
"""

import os
import shutil
import subprocess
import sys
import tempfile

from build_utils import download_file, run_cmd, section

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PIXI_VERSION = "0.44.0"

_PIXI_URLS = {
    "win32": (
        f"https://github.com/prefix-dev/pixi/releases/download/"
        f"v{PIXI_VERSION}/pixi-x86_64-pc-windows-msvc.exe"
    ),
    "darwin_arm64": (
        f"https://github.com/prefix-dev/pixi/releases/download/"
        f"v{PIXI_VERSION}/pixi-aarch64-apple-darwin"
    ),
    "darwin_x86_64": (
        f"https://github.com/prefix-dev/pixi/releases/download/"
        f"v{PIXI_VERSION}/pixi-x86_64-apple-darwin"
    ),
}


def get_pixi_url():
    """Return the platform-appropriate pixi download URL."""
    if sys.platform == "win32":
        return _PIXI_URLS["win32"]
    if sys.platform == "darwin":
        import platform
        machine = platform.machine()
        if machine == "arm64":
            return _PIXI_URLS["darwin_arm64"]
        return _PIXI_URLS["darwin_x86_64"]
    raise RuntimeError(f"Unsupported platform: {sys.platform}")


def get_pixi_binary_name():
    """Return the pixi binary filename for the current platform."""
    if sys.platform == "win32":
        return "pixi.exe"
    return "pixi"


# ---------------------------------------------------------------------------
# Derived paths
# ---------------------------------------------------------------------------
def _repo_root():
    """Return the repository root (two levels up from this file)."""
    return os.path.normpath(
        os.path.join(os.path.dirname(__file__), "..", "..")
    )


def _conda_root():
    """Return the path to the ``conda/`` directory in the repo."""
    return os.path.join(_repo_root(), "conda")


def _conda_envs_dir():
    """Return the path to ``conda/envs/``."""
    return os.path.join(_conda_root(), "envs")


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def pixi_installation(pixi_tmpdir):
    """Download the pixi binary into *pixi_tmpdir*.

    Creates the directory if it does not exist.  Returns the full path
    to the downloaded binary.

    Replaces ``:pixi_installation`` in ``windows_library.bat``.
    """
    with section("Install pixi"):
        os.makedirs(pixi_tmpdir, exist_ok=True)
        binary_name = get_pixi_binary_name()
        dest = os.path.join(pixi_tmpdir, binary_name)
        download_file(get_pixi_url(), dest)
        return dest


def pixi_create_environment(source_dir, project_path, pixi_binary):
    """Create a pixi environment by copying config and running install.

    Copies ``pixi.toml`` and ``pixi.lock`` from *source_dir* into
    *project_path*, then runs ``pixi install --locked``.

    Replaces ``:pixi_create_bootstrap_environment`` and
    ``:pixi_create_gz_environment`` in ``windows_library.bat``.

    Args:
        source_dir: Directory containing ``pixi.toml`` / ``pixi.lock``.
        project_path: Target directory for the pixi project.
        pixi_binary: Path to the pixi binary.
    """
    if not os.path.isdir(source_dir):
        raise FileNotFoundError(
            f"Cannot find {source_dir} directory in the system"
        )

    if os.path.exists(project_path):
        shutil.rmtree(project_path)
    os.makedirs(project_path)

    for fname in ("pixi.toml", "pixi.lock"):
        src = os.path.join(source_dir, fname)
        if os.path.exists(src):
            shutil.copy2(src, project_path)

    pixi_cmd(project_path, pixi_binary, "install", "--locked")


def pixi_load_shell(project_path, pixi_binary):
    """Activate a pixi environment by capturing its shell hook output.

    On Windows: runs ``pixi shell-hook --locked``, writes the output to
    a temporary ``.bat`` file, executes it via ``cmd /c set`` to capture
    the resulting environment variables, and returns them as a dict.

    On macOS (future): would capture bash export statements instead.

    Replaces ``:pixi_load_bootstrap_shell`` and ``:pixi_load_shell``
    in ``windows_library.bat``.

    Args:
        project_path: The pixi project directory.
        pixi_binary: Path to the pixi binary.

    Returns:
        A dict of environment variables produced by the shell hook.
    """
    with section("Load pixi shell"):
        # Get shell-hook output
        result = subprocess.run(
            [pixi_binary, "shell-hook", "--locked"],
            cwd=project_path,
            capture_output=True,
            text=True,
            check=True,
        )
        hook_script = result.stdout
        print(hook_script, flush=True)

        if sys.platform == "win32":
            return _load_shell_windows(hook_script, project_path)
        else:
            return _load_shell_posix(hook_script, project_path)


def _load_shell_windows(hook_script, project_path):
    """Execute hook as a .bat and capture the resulting environment."""
    hook_file = os.path.join(project_path, "hooks.bat")
    with open(hook_file, "w") as f:
        f.write(hook_script)

    # Run hooks.bat then capture environment with 'set'
    capture_cmd = f'call "{hook_file}" && set'
    result = subprocess.run(
        ["cmd", "/c", capture_cmd],
        capture_output=True,
        text=True,
        check=True,
    )

    env = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            key, _, value = line.partition("=")
            env[key] = value
    return env


def _load_shell_posix(hook_script, project_path):
    """Execute hook in bash and capture the resulting environment."""
    hook_file = os.path.join(project_path, "hooks.sh")
    with open(hook_file, "w") as f:
        f.write(hook_script)

    result = subprocess.run(
        ["bash", "-c", f"source '{hook_file}' && env"],
        capture_output=True,
        text=True,
        check=True,
    )

    env = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            key, _, value = line.partition("=")
            env[key] = value
    return env


def pixi_cmd(project_path, pixi_binary, *args):
    """Run an arbitrary pixi subcommand inside *project_path*.

    Uses pushd/popd semantics (cwd) to avoid Windows permission issues
    with ``--manifest-file``.

    Replaces ``:pixi_cmd`` and ``:pixi_bootstrap_cmd`` in
    ``windows_library.bat``.
    """
    cmd = [pixi_binary] + list(args)
    run_cmd(cmd, cwd=project_path)


def detect_conda_env(package_name, major_version, yaml_file=None):
    """Detect the conda environment name for a package and version.

    Imports and calls ``find_conda_configs()`` from the existing
    detection script.

    Args:
        package_name: e.g. ``"gz-rendering"``.
        major_version: e.g. ``8``.
        yaml_file: Path to ``gz-collections.yaml``.  Defaults to
            ``jenkins-scripts/dsl/gz-collections.yaml`` in the repo.

    Returns:
        The conda environment version string (e.g. ``"legacy"``).

    Raises:
        SystemExit: If detection fails or returns ambiguous results.
    """
    if yaml_file is None:
        yaml_file = os.path.join(
            _repo_root(), "jenkins-scripts", "dsl", "gz-collections.yaml"
        )

    # Import the detection module from the dsl/tools directory
    tools_dir = os.path.join(
        _repo_root(), "jenkins-scripts", "dsl", "tools"
    )
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)

    from get_ciconfigs_from_package_and_version import find_conda_configs

    result = find_conda_configs(package_name, major_version, yaml_file)

    if not result["found"]:
        sys.exit(result["message"])

    if not result["conda_configs"]:
        sys.exit(
            f"No conda configurations found for "
            f"{package_name} v{major_version}"
        )

    if len(result["conda_configs"]) > 1:
        names = ", ".join(c["name"] for c in result["conda_configs"])
        sys.exit(
            f"Multiple conda configurations found for "
            f"{package_name} v{major_version}: {names}"
        )

    env_name = result["conda_configs"][0]["version"]
    print(f"Detected conda environment: {env_name}", flush=True)
    return env_name
