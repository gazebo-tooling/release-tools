"""Per-OS facts. Phase A knows Windows only."""

import ntpath
import os
import stat
import sys
from pathlib import Path

from . import envvars
from .runner import CIError, section


def read_report(path):
    """Text of a tool report that may be UTF-16 (BOM) or UTF-8."""
    data = Path(path).read_bytes()
    if data[:2] in (b"\xff\xfe", b"\xfe\xff"):
        return data.decode("utf-16")
    return data.decode("utf-8", errors="replace")


class WindowsPlatform:
    detect_os = "windows"          # system.so in gz-collections.yaml
    detect_arch = "amd64"          # system.arch in gz-collections.yaml
    default_build_type = "Release"
    hooks_file = "hooks.bat"
    hook_shell = "cmd"

    def normalize_key(self, name):
        # Windows variable names are case-insensitive
        return name.upper()

    def normalize_env(self, environ):
        return {self.normalize_key(k): v for k, v in environ.items()}

    def references(self, value):
        return envvars.windows_references(value)

    def expand(self, value, env):
        return envvars.expand_windows(value, env)

    def _pixi_root(self, env):
        if not env.get("PROGRAMDATA"):
            raise CIError("PROGRAMDATA is not set")
        return Path(env["PROGRAMDATA"]) / "pixi"

    def default_pixi_exe(self, env):
        return self._pixi_root(env) / "pixi.exe"

    def default_project_path(self, env):
        return self._pixi_root(env) / "project"

    def package_build_env(self, env):
        """Environment of the build of the package under test.

        With Visual Studio generators colcon-cmake runs "cmake --build"
        without --parallel, so MSBuild builds one project at a time (the /MP
        that colcon-cmake adds to CL only compiles the files of one project
        in parallel). CMake passes CMAKE_BUILD_PARALLEL_LEVEL to MSBuild as
        /m:N. Only the package under test uses it: colcon already builds
        several dependencies at the same time, and a job can still define it
        to apply it to the dependencies too.
        """
        env = dict(env)
        if not env.get("CMAKE_BUILD_PARALLEL_LEVEL"):
            env["CMAKE_BUILD_PARALLEL_LEVEL"] = (
                env.get("NUMBER_OF_PROCESSORS") or str(os.cpu_count() or 1))
        # Do not leave MSBuild worker nodes running after the build
        env["MSBUILDDISABLENODEREUSE"] = "1"
        return env

    def post_activation(self, env):
        prefix = env.get("CONDA_PREFIX")
        if not prefix:
            raise CIError("CONDA_PREFIX is not set after the pixi activation")
        env = dict(env)
        env["OGRE_RESOURCE_PATH"] = ntpath.join(prefix, "Library", "bin")
        env["OGRE2_RESOURCE_PATH"] = ntpath.join(prefix, "Library", "bin",
                                                 "OGRE-Next")
        return env

    def is_hidden_or_system(self, path):
        # xcopy without /h skips them; Git for Windows hides .git
        attributes = getattr(os.lstat(path), "st_file_attributes", 0)
        return bool(attributes & (stat.FILE_ATTRIBUTE_HIDDEN |
                                  stat.FILE_ATTRIBUTE_SYSTEM))

    def check_gpu(self, runner, cfg, env):
        report = cfg.workspace / "dxdiag.txt"
        with section("dxdiag info"):
            runner.run("dxdiag", ["/t", report], cwd=cfg.workspace, env=env,
                       check=False)
            if not report.is_file():
                raise CIError(f"dxdiag did not write {report}")
            text = read_report(report)
            print(text, flush=True)
            print(f"Checking for correct NVIDIA GPU support {report}",
                  flush=True)
            if "Manufacturer: NVIDIA" not in text:
                raise CIError("NVIDIA GPU not found in dxdiag")


def current():
    if sys.platform == "win32":
        return WindowsPlatform()
    raise CIError(f"pixi_ci does not support {sys.platform} yet")
