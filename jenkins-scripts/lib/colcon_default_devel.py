"""Main orchestrator for colcon-based conda/pixi builds.

Single entry point replacing all per-package ``.bat`` entry points and
``colcon-default-devel-windows.bat``.  Accepts CLI arguments instead of
environment variables, making the interface explicit and testable.

Usage::

    python colcon_default_devel.py \\
        --package gz-rendering \\
        --auto-major-version \\
        --gpu-support \\
        --conda-env-name legacy_ogre23
"""

import argparse
import os
import shutil
import sys
import tempfile

# Ensure the lib directory is on sys.path so sibling modules resolve.
_LIB_DIR = os.path.dirname(os.path.abspath(__file__))
if _LIB_DIR not in sys.path:
    sys.path.insert(0, _LIB_DIR)

from build_utils import detect_cmake_major_version, run_cmd, section
from colcon_utils import (
    build_workspace,
    get_source_from_gazebodistro,
    list_workspace_pkgs,
    resolve_colcon_package_name,
    run_tests,
)
from pixi_utils import (
    _conda_envs_dir,
    _conda_root,
    detect_conda_env,
    pixi_cmd,
    pixi_create_environment,
    pixi_installation,
    pixi_load_shell,
)


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Colcon-based conda/pixi build orchestrator for Gazebo CI",
    )
    parser.add_argument(
        "--package", required=True,
        help="Colcon package name (e.g. gz-rendering)",
    )
    parser.add_argument(
        "--vcs-directory", default=None,
        help="Relative path inside WORKSPACE containing sources "
             "(default: same as --package)",
    )
    parser.add_argument(
        "--auto-major-version", action="store_true", default=False,
        help="Auto-detect major version from CMakeLists.txt",
    )
    parser.add_argument(
        "--build-type", default="Release",
        help="CMake build type (default: Release)",
    )
    parser.add_argument(
        "--enable-tests", dest="enable_tests",
        action="store_true", default=True,
    )
    parser.add_argument(
        "--disable-tests", dest="enable_tests",
        action="store_false",
    )
    parser.add_argument(
        "--conda-env-name", default=None,
        help="Conda environment name (auto-detected if omitted)",
    )
    parser.add_argument(
        "--keep-workspace", action="store_true", default=False,
        help="Do not clean the workspace after build",
    )
    parser.add_argument(
        "--gazebodistro-file", default=None,
        help="vcs yaml file in the gazebodistro repo (auto-derived if omitted)",
    )
    parser.add_argument(
        "--extra-cmake-args", default=None,
        help="Extra CMake arguments for the target package",
    )
    parser.add_argument(
        "--gpu-support", action="store_true", default=False,
        help="Require NVIDIA GPU (runs dxdiag check on Windows)",
    )
    parser.add_argument(
        "--platform", default="x86_amd64",
        help="MSVC platform target (default: x86_amd64)",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)

    workspace = os.environ.get("WORKSPACE", os.getcwd())
    vcs_directory = args.vcs_directory or args.package
    package_original = args.package

    # -------------------------------------------------------------------
    # Validate inputs
    # -------------------------------------------------------------------
    vcs_source = os.path.join(workspace, vcs_directory)
    if not os.path.isdir(vcs_source):
        sys.exit(f"VCS_DIRECTORY {vcs_source} does not exist")

    # -------------------------------------------------------------------
    # Platform-specific: configure compiler
    # -------------------------------------------------------------------
    if sys.platform == "win32":
        from windows_utils import configure_msvc_compiler
        configure_msvc_compiler(args.platform)

    # -------------------------------------------------------------------
    # Platform-specific: GPU check
    # -------------------------------------------------------------------
    if args.gpu_support:
        if sys.platform == "win32":
            from windows_utils import check_gpu_support
            check_gpu_support(workspace)

    # -------------------------------------------------------------------
    # Install pixi
    # -------------------------------------------------------------------
    pixi_tmpdir = os.path.join(tempfile.gettempdir(), "pixi")
    pixi_binary = pixi_installation(pixi_tmpdir)

    # -------------------------------------------------------------------
    # Create and load bootstrap environment
    # -------------------------------------------------------------------
    bootstrap_project = os.path.join(pixi_tmpdir, "bootstrap_project")
    bootstrap_source = os.path.join(_conda_root(), "config-detector")

    with section("pixi: create bootstrap environment"):
        pixi_create_environment(bootstrap_source, bootstrap_project,
                                pixi_binary)

    bootstrap_env = pixi_load_shell(bootstrap_project, pixi_binary)
    os.environ.update(bootstrap_env)

    # -------------------------------------------------------------------
    # Auto-detect major version
    # -------------------------------------------------------------------
    major_version = ""
    colcon_package = package_original
    if args.auto_major_version:
        cmakelists = os.path.join(vcs_source, "CMakeLists.txt")
        major_version = detect_cmake_major_version(cmakelists)
        colcon_package = f"{package_original}{major_version}"
        print(f"MAJOR_VERSION detected: {major_version}", flush=True)

    # -------------------------------------------------------------------
    # Auto-detect or validate conda environment
    # -------------------------------------------------------------------
    conda_env_name = args.conda_env_name
    if not conda_env_name:
        with section("auto-detect conda environment"):
            conda_env_name = detect_conda_env(
                package_original, int(major_version) if major_version else 0,
            )
    else:
        print(f"Using user-specified conda environment: {conda_env_name}",
              flush=True)

    # -------------------------------------------------------------------
    # Create and load Gazebo conda environment
    # -------------------------------------------------------------------
    pixi_project_path = os.environ.get(
        "PIXI_PROJECT_PATH",
        os.path.join(pixi_tmpdir, "project"),
    )
    conda_env_source = os.path.join(_conda_envs_dir(), conda_env_name)

    with section(f"pixi: create {conda_env_name} environment"):
        pixi_create_environment(conda_env_source, pixi_project_path,
                                pixi_binary)

    with section("pixi: info"):
        pixi_cmd(pixi_project_path, pixi_binary, "info")

    with section("pixi: list packages"):
        pixi_cmd(pixi_project_path, pixi_binary, "list")

    gz_env = pixi_load_shell(pixi_project_path, pixi_binary)
    os.environ.update(gz_env)

    # -------------------------------------------------------------------
    # Platform-specific: set OGRE environment variables
    # -------------------------------------------------------------------
    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    if not conda_prefix:
        sys.exit("CONDA_PREFIX is not set after loading pixi shell")

    with section("pixi: custom environment variable for gz"):
        if sys.platform == "win32":
            from windows_utils import get_ogre_env_vars
            ogre_vars = get_ogre_env_vars(conda_prefix)
        else:
            # macOS paths would go here in the future
            ogre_vars = {}
        os.environ.update(ogre_vars)

    # -------------------------------------------------------------------
    # Setup workspace
    # -------------------------------------------------------------------
    local_ws = os.path.join(workspace, "ws")
    local_ws_src = os.path.join(local_ws, "src")
    local_ws_software_dir = os.path.join(local_ws_src, vcs_directory)

    with section("setup workspace"):
        if not args.keep_workspace and os.path.isdir(local_ws):
            with section("preclean workspace"):
                shutil.rmtree(local_ws)
        os.makedirs(local_ws_src, exist_ok=True)

    # -------------------------------------------------------------------
    # Get sources from gazebodistro
    # -------------------------------------------------------------------
    distro_file = args.gazebodistro_file
    if not distro_file:
        distro_file = f"{vcs_directory}{major_version}.yaml"
    else:
        print(f"Using user defined GAZEBODISTRO_FILE: {distro_file}",
              flush=True)

    gazebodistro_branch = os.environ.get("GAZEBODISTRO_BRANCH", "master")
    pr_branch = os.environ.get("ghprbSourceBranch")

    with section(f"get open robotics deps ({distro_file}) sources "
                 f"into the workspace"):
        get_source_from_gazebodistro(
            distro_file, local_ws_src,
            branch=gazebodistro_branch, pr_branch=pr_branch,
        )

    # -------------------------------------------------------------------
    # Move package source into workspace (overwrites gazebodistro copy)
    # -------------------------------------------------------------------
    with section(f"move {vcs_directory} source to workspace"):
        if os.path.isdir(local_ws_software_dir):
            shutil.rmtree(local_ws_software_dir)
        shutil.copytree(vcs_source, local_ws_software_dir)

    # -------------------------------------------------------------------
    # List packages & resolve colcon name
    # -------------------------------------------------------------------
    os.chdir(local_ws)

    with section("packages in workspace"):
        list_workspace_pkgs()

    colcon_package = resolve_colcon_package_name(
        package_original, major_version, local_ws,
    )

    # -------------------------------------------------------------------
    # Build workspace
    # -------------------------------------------------------------------
    with section(f"compiling {vcs_directory}"):
        build_workspace(
            colcon_package,
            build_type=args.build_type,
            extra_cmake_args=args.extra_cmake_args,
        )

    # -------------------------------------------------------------------
    # Run tests & export results
    # -------------------------------------------------------------------
    if args.enable_tests:
        test_result_path = os.path.join(
            local_ws, "build", colcon_package, "test_results",
        )
        export_path = os.path.join(workspace, "build", "test_results")

        run_tests(colcon_package)

        with section("export testing results"):
            if os.path.isdir(export_path):
                shutil.rmtree(export_path)
            if os.path.isdir(test_result_path):
                shutil.copytree(test_result_path, export_path)

    # -------------------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------------------
    if not args.keep_workspace:
        with section("clean up workspace"):
            os.chdir(workspace)
            shutil.rmtree(local_ws)


if __name__ == "__main__":
    main()
