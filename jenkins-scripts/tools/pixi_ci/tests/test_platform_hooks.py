"""The platform hooks around the activation and the build: where run_job
calls them and what they change in the colcon command lines."""

from fakes import find, make_job
from pixi_ci.platforms import WindowsPlatform

HANDLERS = ["--event-handler", "console_cohesion+", "desktop_notification-"]


class HookedPlatform(WindowsPlatform):
    cmake_args = ("-DHOOK=1",)

    def isolate_env(self, env):
        return {k: v for k, v in env.items() if k != "DROPPED"}

    def dependency_cxx_flags(self, env):
        return "-O1 -w"

    def report_toolchain(self, runner, cfg, env):
        runner.run("report-toolchain", [], cwd=cfg.workspace, env=env)

    def check_build(self, runner, cfg, env):
        runner.run("check-build", [], cwd=cfg.workspace, env=env)


def cmake_args(call):
    return call.args[call.args.index("--cmake-args"):]


def test_platform_hooks_shape_the_job(tmp_path):
    job = make_job(tmp_path, platform=HookedPlatform())
    code, calls = job.run(DROPPED="1")
    tools = [c.tool for c in calls]
    dependencies, package = find(calls, "colcon", "build")
    json_hook = calls.index(find(calls, job.pixi, "shell-hook", "--json")[0])
    check = tools.index("check-build")
    assert code == 0
    assert not any("DROPPED" in c.env for c in calls)
    assert calls[json_hook + 1].tool == "report-toolchain"
    assert cmake_args(dependencies) == [
        "--cmake-args", " -DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=0",
        "-DCMAKE_CXX_FLAGS=-O1 -w", "-DHOOK=1"] + HANDLERS
    assert cmake_args(package) == [
        "--cmake-args", " -DCMAKE_BUILD_TYPE=Release", "-DHOOK=1",
        " -DBUILD_TESTING=1"] + HANDLERS
    assert calls[check - 1] is package
    assert calls[check + 1].args[:1] == ["test"]


def test_check_build_runs_without_tests(tmp_path):
    job = make_job(tmp_path, platform=HookedPlatform())
    code, calls = job.run(ENABLE_TESTS="false")
    assert code == 0
    assert calls[-1].tool == "check-build"
    assert calls[-2] is find(calls, "colcon", "build")[1]
