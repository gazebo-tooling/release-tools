"""colcon build, test and the JUnit export."""

import shutil
from dataclasses import dataclass

from .fsops import remove_tree
from .runner import CIError, section


@dataclass(frozen=True)
class BuildSetup:
    """What the generator changes in the two colcon builds."""
    deps_env: dict
    package_env: dict
    # CMake arguments of both builds, e.g. the generator
    cmake_args: tuple = ()
    # CMake arguments that turn off the warnings of the dependencies
    quiet_deps_args: tuple = ("-DCMAKE_CXX_FLAGS=-w",)


def build(runner, cfg, package, setup, watch):
    """Dependencies without tests, then the package with tests, in the
    environments of setup, a BuildSetup.

    watch(label) is a context manager reporting the memory used while each
    of the two builds runs.
    """
    common = ["build", "--build-base", "build", "--install-base", "install",
              "--parallel-workers", str(cfg.make_jobs)]
    # The leading space is what today's .bat passes; colcon needs it for
    # values starting with a dash only, kept for parity
    build_type = (["--cmake-args", f" -DCMAKE_BUILD_TYPE={cfg.build_type}"]
                  + list(setup.cmake_args))
    handlers = ["--event-handler", "console_cohesion+", "desktop_notification-"]
    with section("colcon compilation without test for dependencies of "
                 f"{package}"), watch("dependencies"):
        runner.run("colcon", common + ["--packages-skip", package] + build_type
                   + ["-DBUILD_TESTING=0"] + list(setup.quiet_deps_args)
                   + handlers,
                   cwd=cfg.ws, env=setup.deps_env)
    with section(f"colcon compilation with tests for {package}"), \
            watch(package):
        runner.run("colcon", common + ["--packages-select", package]
                   + build_type + list(cfg.extra_cmake_args)
                   + [" -DBUILD_TESTING=1"] + handlers,
                   cwd=cfg.ws, env=setup.package_env)


def test(runner, cfg, package, env):
    """Test failures do not fail the job: Jenkins reads the JUnit files."""
    with section(f"colcon test for {package}"):
        runner.run("colcon", ["test", "--install-base", "install",
                              "--packages-select", package,
                              "--executor", "sequential",
                              "--event-handler", "console_direct+",
                              "desktop_notification-"],
                   cwd=cfg.ws, env=env, check=False)
    with section("colcon test-result"):
        runner.run("colcon", ["test-result", "--all"], cwd=cfg.ws, env=env,
                   check=False)


def export_test_results(cfg, package):
    source = cfg.ws / "build" / package / "test_results"
    destination = cfg.workspace / "build" / "test_results"
    remove_tree(destination)
    if not source.is_dir():
        raise CIError(f"no test results in {source}")
    shutil.copytree(source, destination)
