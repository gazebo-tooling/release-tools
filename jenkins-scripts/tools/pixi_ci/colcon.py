"""colcon build, test and the JUnit export."""

import shutil

from .fsops import remove_tree
from .runner import CIError, section


def with_makeflags(env, make_jobs):
    return dict(env, MAKEFLAGS=f"-j{make_jobs}")


def build(runner, cfg, package, env):
    """Dependencies without tests, then the package with tests."""
    common = ["build", "--build-base", "build", "--install-base", "install",
              "--parallel-workers", str(cfg.make_jobs)]
    # The leading space is what today's .bat passes; colcon needs it for
    # values starting with a dash only, kept for parity
    build_type = ["--cmake-args", f" -DCMAKE_BUILD_TYPE={cfg.build_type}"]
    handlers = ["--event-handler", "console_cohesion+", "desktop_notification-"]
    with section("colcon compilation without test for dependencies of "
                 f"{package}"):
        runner.run("colcon", common + ["--packages-skip", package] + build_type
                   + ["-DBUILD_TESTING=0", "-DCMAKE_CXX_FLAGS=-w"] + handlers,
                   cwd=cfg.ws, env=env)
    with section(f"colcon compilation with tests for {package}"):
        runner.run("colcon", common + ["--packages-select", package]
                   + build_type + list(cfg.extra_cmake_args)
                   + [" -DBUILD_TESTING=1"] + handlers,
                   cwd=cfg.ws, env=env)


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
