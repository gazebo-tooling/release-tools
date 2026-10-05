"""pixi_ci entry point; Jenkins runs it through ../run_pixi_ci.py."""

import argparse
import os
import sys

from . import colcon, envs, platforms, workspace
from .config import RELEASE_TOOLS, JobConfig
from .fsops import remove_tree
from .runner import CIError, Runner, section


def run_job(cfg, platform, runner, base_env):
    if cfg.gpu_needed:
        platform.check_gpu(runner, cfg, base_env)
    major = envs.detect_major_version(runner, cfg, base_env)
    env_name = envs.select_env(runner, cfg, platform, major, base_env)
    if cfg.reuse_pixi:
        envs.check_reused_project(cfg)
    else:
        with section(f"pixi: create {env_name} environment"):
            envs.install_env(runner, cfg, env_name, base_env)
    envs.show_env(runner, cfg, base_env)
    with section("pixi: enable shell"):
        env = envs.activate(runner, cfg, platform, base_env)
    with section("pixi: custom environment variable for gz"):
        env = platform.post_activation(env)

    workspace.setup(cfg)
    if cfg.gazebodistro_file is not None:
        print(f"Using user defined GAZEBODISTRO_FILE: {cfg.gazebodistro_file}",
              flush=True)
    distro_file = cfg.gazebodistro_file or f"{cfg.vcs_directory}{major}.yaml"
    with section(f"get open robotics deps ({distro_file}) sources into the "
                 "workspace"):
        workspace.import_gazebodistro(runner, cfg, distro_file, env)
    workspace.copy_sources_under_test(cfg, platform)
    package = workspace.find_package(runner, cfg, major, env)

    env = colcon.with_makeflags(env, cfg.make_jobs)
    with section(f"compiling {cfg.vcs_directory}"):
        colcon.build(runner, cfg, package, env)
    if cfg.enable_tests:
        with section(f"running tests for {package}"):
            colcon.test(runner, cfg, package, env)
        with section("export testing results"):
            colcon.export_test_results(cfg, package)
    if not cfg.keep_workspace:
        with section("clean up workspace"):
            remove_tree(cfg.ws)


def main(argv=None, environ=None, runner=None, platform=None,
         release_tools=RELEASE_TOOLS):
    parser = argparse.ArgumentParser(
        prog="pixi_ci",
        description="Build and test a Gazebo library with pixi and colcon")
    parser.add_argument("library",
                        help="gz-collections.yaml library name, e.g. gz-sim")
    args = parser.parse_args(argv)
    try:
        platform = platform or platforms.current()
        base_env = platform.normalize_env(os.environ if environ is None
                                          else environ)
        cfg = JobConfig.from_environ(args.library, base_env, platform,
                                     release_tools)
        with section("pixi_ci configuration"):
            print("\n".join(cfg.describe()), flush=True)
        run_job(cfg, platform, runner or Runner(), base_env)
    except CIError as error:
        print(f"ERROR: {error}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
