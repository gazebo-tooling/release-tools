"""Fake runner and job for the recorded-command tests: the driver runs
end to end and the runner records each command instead of running it."""

import json
import shutil
from dataclasses import dataclass, field
from pathlib import Path

from pixi_ci.__main__ import main
from pixi_ci.config import RELEASE_TOOLS
from pixi_ci.platforms import WindowsPlatform
from pixi_ci.runner import PYTHON, CIError, Result

PREFIX = r"C:\pixi\project\.pixi\envs\default"
HOOKS = '@SET "CONDA_PREFIX=C:\\pixi\\project\\.pixi\\envs\\default"\n'
# pixi shell-hook --shell <shell> output
SHELL_HOOKS = {"cmd": HOOKS,
               "bash": 'export CONDA_PREFIX="/pixi/project/.pixi/envs/default"\n'}


@dataclass
class Call:
    tool: str
    args: list
    cwd: Path
    env: dict


class FakeRunner:
    def __init__(self, respond):
        self.calls = []
        self._respond = respond

    def run(self, tool, args, *, cwd, env, check=True, capture=False):
        call = Call(str(tool), [str(a) for a in args], Path(cwd), dict(env))
        self.calls.append(call)
        result = self._respond(call) or Result(0)
        if check and result.returncode != 0:
            raise CIError(f"exit code {result.returncode}: "
                          f"{' '.join([call.tool] + call.args)}")
        return result


@dataclass
class Job:
    tmp: Path
    library: str
    major: str
    colcon_names: tuple
    environ: dict
    dxdiag: str = "Card name: NVIDIA RTX A2000\nManufacturer: NVIDIA\n"
    distro_files: tuple = None
    # (predicate(call), Result) pairs answered before the defaults below
    replies: list = field(default_factory=list)
    platform: object = field(default_factory=WindowsPlatform)
    # The pixi shell-hook --json variables; None gives the Windows ones
    activation: dict = None

    @property
    def workspace(self):
        return self.tmp / "workspace"

    @property
    def rt(self):
        return self.tmp / "release-tools"

    @property
    def pixi(self):
        return str(self.platform.default_pixi_exe(
            self.platform.normalize_env(self.environ)))

    @property
    def project(self):
        return self.platform.default_project_path(
            self.platform.normalize_env(self.environ))

    def activated_variables(self):
        if self.activation is not None:
            return self.activation
        # pixi prints the variables in a different order on every call
        return {
            "QT_QPA_PLATFORM_PLUGIN_PATH": r"%CONDA_PREFIX%\Library\plugins",
            "CONDA_PREFIX": PREFIX,
            "PATH": PREFIX + r"\Library\bin;" + self.environ["PATH"],
        }

    def respond(self, call):
        for predicate, result in self.replies:
            if predicate(call):
                return result
        script = Path(call.args[0]).name if call.args else ""
        if call.tool == PYTHON and script == "detect_cmake_major_version.py":
            return Result(0, self.major + "\n")
        if call.tool == PYTHON and script == "get_ciconfigs_from_package_and_version.py":
            return Result(0, "noble_like\n")
        if call.tool == PYTHON and script == "detect_ci_matching_branch.py":
            return Result(0 if "ci_matching_branch/" in call.args[1] else 1)
        if call.tool == "dxdiag":
            Path(call.args[1]).write_text(self.dxdiag)
        if call.tool == self.pixi and call.args[:2] == ["shell-hook", "--locked"]:
            return Result(0, SHELL_HOOKS[option(call.args, "--shell")])
        if call.tool == self.pixi and call.args[:2] == ["shell-hook", "--json"]:
            return Result(0, json.dumps(
                {"environment_variables": self.activated_variables()}))
        if call.tool == "git" and call.args[0] == "clone":
            Path(call.args[2]).mkdir(parents=True)
            for name in self.distro_files or (f"{self.library}{self.major}.yaml",):
                (Path(call.args[2]) / name).write_text("repositories: {}\n")
        if call.tool == "colcon" and call.args[:2] == ["list", "--names-only"]:
            return Result(0, "\n".join(self.colcon_names) + "\n")
        if call.tool == "colcon" and call.args[0] == "test":
            package = option(call.args, "--packages-select")
            results = call.cwd / "build" / package / "test_results" / package
            results.mkdir(parents=True)
            (results / "UNIT_Vector3_TEST.xml").write_text("<testsuites/>")
        return None

    def run(self, **variables):
        """Run the driver; return (exit code, calls)."""
        runner = FakeRunner(self.respond)
        environ = dict(self.environ, **variables)
        code = main([self.library], environ=environ, runner=runner,
                    platform=self.platform, release_tools=self.rt)
        return code, runner.calls


def make_job(tmp_path, library="gz-math", major="9",
             colcon_names=("gz-cmake", "gz-utils", "gz-math"), **kwargs):
    for env_name in ("noble_like", "legacy"):
        env_dir = tmp_path / "release-tools" / "conda" / "envs" / env_name
        env_dir.mkdir(parents=True)
        (env_dir / "pixi.toml").write_text("[workspace]\n")
        (env_dir / "pixi.lock").write_text("version: 6\n")
    dsl = tmp_path / "release-tools" / "jenkins-scripts" / "dsl"
    dsl.mkdir(parents=True)
    shutil.copy(RELEASE_TOOLS / "jenkins-scripts" / "dsl" / "gz-collections.yaml",
                dsl)
    sources = tmp_path / "workspace" / library
    (sources / ".git").mkdir(parents=True)
    (sources / "CMakeLists.txt").write_text(f"project({library} VERSION {major}.0.0)\n")
    environ = {"WORKSPACE": str(tmp_path / "workspace"),
               "PROGRAMDATA": str(tmp_path / "ProgramData"),
               "PATH": r"C:\Windows\system32",
               "MAKE_JOBS": "4"}
    return Job(tmp_path, library, major, tuple(colcon_names), environ, **kwargs)


def commands(calls):
    return [(c.tool, c.args, c.cwd) for c in calls]


def find(calls, tool, *first_args):
    """The calls of tool whose arguments start with first_args."""
    return [c for c in calls
            if c.tool == tool and c.args[:len(first_args)] == list(first_args)]


def option(args, flag):
    return args[args.index(flag) + 1]


def sections(capsys):
    return [line[len("# BEGIN SECTION: "):]
            for line in capsys.readouterr().out.splitlines()
            if line.startswith("# BEGIN SECTION: ")]
