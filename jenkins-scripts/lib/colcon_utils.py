"""Colcon build, test, and workspace management utilities.

Handles gazebodistro source fetching, colcon build/test operations,
workspace listing, and the complex gz/ignition package name resolution.
Platform-independent.
"""

import os
import re
import shutil
import subprocess
import sys

from build_utils import run_cmd, section


def get_source_from_gazebodistro(distro_file, dest_dir,
                                 branch="master", pr_branch=None):
    """Clone gazebodistro repo and import sources via vcs.

    Handles ``ci_matching_branch`` detection: if *pr_branch* starts
    with ``ci_matching_branch/``, the matching branch is checked out
    from the gazebodistro clone before importing.

    Replaces ``:get_source_from_gazebodistro`` in ``windows_library.bat``.

    Args:
        distro_file: Name of the yaml file inside the gazebodistro repo.
        dest_dir: Destination directory for ``vcs import``.
        branch: Branch to clone from gazebodistro (default ``"master"``).
        pr_branch: Value of ``ghprbSourceBranch`` env var (may be None).
    """
    gzdistro_dir = "gazebodistro"
    if os.path.isdir(gzdistro_dir):
        shutil.rmtree(gzdistro_dir)

    run_cmd([
        "git", "clone",
        "https://github.com/gazebo-tooling/gazebodistro",
        gzdistro_dir, "-b", branch,
    ])

    # Check if ci_matching_branch name is used
    if pr_branch:
        if re.search(r"ci_matching_branch/", pr_branch):
            print(f"trying to checkout branch {pr_branch} from gazebodistro",
                  flush=True)
            run_cmd(["git", "-C", gzdistro_dir,
                     "fetch", "origin", pr_branch], check=False)
            run_cmd(["git", "-C", gzdistro_dir,
                     "checkout", pr_branch], check=False)
        else:
            print(f"branch name {pr_branch} is not a match", flush=True)
        # print branch for informational purposes
        run_cmd(["git", "-C", gzdistro_dir, "branch"])

    yaml_path = os.path.join(gzdistro_dir, distro_file)
    _vcs_import(yaml_path, dest_dir)
    run_cmd(["vcs", "pull"], check=False)


def _vcs_import(yaml_path, dest_dir):
    """Run ``vcs import`` feeding the yaml file via stdin."""
    with open(yaml_path, "r") as f:
        print(f"+ vcs import --retry 5 --force < {yaml_path} {dest_dir}",
              flush=True)
        subprocess.run(
            ["vcs", "import", "--retry", "5", "--force", dest_dir],
            stdin=f,
            check=True,
        )


def build_workspace(package_name, build_type="Release",
                    extra_cmake_args=None, make_jobs=None):
    """Build the colcon workspace in two phases.

    Phase 1: build all dependencies (skip *package_name*) without tests.
    Phase 2: build *package_name* with tests enabled.

    Replaces ``:build_workspace`` and ``:_colcon_build_cmd`` in
    ``windows_library.bat``.

    Args:
        package_name: The colcon package to test.
        build_type: CMake build type (default ``"Release"``).
        extra_cmake_args: Extra CMake arguments for the target package.
        make_jobs: Parallel job count.  Falls back to ``MAKE_JOBS`` env
            var if not provided.
    """
    if make_jobs is None:
        make_jobs = os.environ.get("MAKE_JOBS", "1")
    make_jobs = str(make_jobs)

    os.environ["MAKEFLAGS"] = f"-j{make_jobs}"

    with section(f"colcon compilation without test for "
                 f"dependencies of {package_name}"):
        _colcon_build_cmd(
            colcon_extra_args=["--packages-skip", package_name],
            cmake_args=[
                f"-DCMAKE_BUILD_TYPE={build_type}",
                "-DBUILD_TESTING=0",
                "-DCMAKE_CXX_FLAGS=-w",
            ],
            make_jobs=make_jobs,
        )

    cmake_args = [
        f"-DCMAKE_BUILD_TYPE={build_type}",
        "-DBUILD_TESTING=1",
    ]
    if extra_cmake_args:
        if isinstance(extra_cmake_args, str):
            cmake_args.append(extra_cmake_args)
        else:
            cmake_args.extend(extra_cmake_args)

    with section(f"colcon compilation with tests for {package_name}"):
        _colcon_build_cmd(
            colcon_extra_args=["--packages-select", package_name],
            cmake_args=cmake_args,
            make_jobs=make_jobs,
        )


def _colcon_build_cmd(colcon_extra_args, cmake_args, make_jobs):
    """Low-level colcon build invocation."""
    cmd = [
        "colcon", "build",
        "--build-base", "build",
        "--install-base", "install",
        "--parallel-workers", make_jobs,
    ] + colcon_extra_args + [
        "--cmake-args",
    ] + cmake_args + [
        "--event-handler", "console_cohesion+",
    ]
    run_cmd(cmd)


def run_tests(package_name):
    """Run colcon tests for *package_name* and print results.

    Replaces ``:tests_in_workspace`` in ``windows_library.bat``.
    """
    with section(f"colcon test for {package_name}"):
        run_cmd([
            "colcon", "test",
            "--install-base", "install",
            "--packages-select", package_name,
            "--executor", "sequential",
            "--event-handler", "console_direct+",
        ], check=False)

    with section("colcon test-result"):
        run_cmd(["colcon", "test-result", "--all"], check=False)


def list_workspace_pkgs():
    """List workspace packages and export VCS versions.

    Replaces ``:list_workspace_pkgs`` in ``windows_library.bat``.
    """
    run_cmd(["colcon", "list", "-t"])
    run_cmd(["vcs", "export", "--exact"])


def resolve_colcon_package_name(package_original, major_version,
                                workspace_dir):
    """Resolve the actual colcon package name in the workspace.

    Tries these names in order:
    1. ``{package_original}{major_version}`` (e.g. ``gz-rendering8``)
    2. ``{package_original}`` without version suffix
    3. Ignition equivalents: ``gz`` -> ``ignition``, ``sim`` -> ``gazebo``
       (special case: ``gz-tools`` v1 -> ``ignition-tools``)

    Replaces the name resolution logic in
    ``colcon-default-devel-windows.bat:161-194``.

    Args:
        package_original: Original package name (e.g. ``"gz-rendering"``).
        major_version: Major version string (may be ``""`` or ``None``).
        workspace_dir: Path to the colcon workspace root.

    Returns:
        The resolved package name that exists in the workspace.

    Raises:
        SystemExit: If the package cannot be found under any name.
    """
    major_version = major_version or ""
    candidates = []

    # 1. package + major version
    if major_version:
        candidates.append(f"{package_original}{major_version}")

    # 2. package without version
    candidates.append(package_original)

    # 3. ignition equivalents
    if major_version:
        # special case: gz-tools v1 -> ignition-tools
        if package_original == "gz-tools" and major_version == "1":
            candidates.append("ignition-tools")
        else:
            ign_name = f"{package_original}{major_version}"
            ign_name = ign_name.replace("gz", "ignition")
            ign_name = ign_name.replace("sim", "gazebo")
            candidates.append(ign_name)

    for candidate in candidates:
        with section(f"Check if package {candidate} is in colcon workspace"):
            result = subprocess.run(
                ["colcon", "list", "--names-only"],
                cwd=workspace_dir,
                capture_output=True,
                text=True,
            )
            pkg_list = result.stdout.strip().splitlines()
            print("Packages in workspace:", flush=True)
            for p in pkg_list:
                print(f"  {p}", flush=True)

            if candidate in pkg_list:
                print(f"Using package name {candidate}", flush=True)
                return candidate

    sys.exit(f"Failed to find package {package_original} "
             f"(tried: {', '.join(candidates)}) in workspace.")
