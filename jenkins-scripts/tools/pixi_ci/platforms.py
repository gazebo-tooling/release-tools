"""Per-OS facts. Phase A knows Windows only."""

import contextlib
import ntpath
import os
import stat
import sys
from pathlib import Path

from . import colcon, envvars, jobserver, memory
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

    def memory_probe(self):
        return memory.WindowsMemoryProbe()

    def _cpus(self, env):
        return env.get("NUMBER_OF_PROCESSORS") or str(os.cpu_count() or 1)

    @contextlib.contextmanager
    def build_setup(self, env, cfg):
        """Yield the colcon.BuildSetup of the dependencies and the package."""
        if cfg.cmake_generator == "Ninja":
            with self._ninja_setup(env, cfg) as setup:
                yield setup
        else:
            yield colcon.BuildSetup(deps_env=env,
                                    package_env=self._msbuild_package_env(env))

    def _msbuild_package_env(self, env):
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
            env["CMAKE_BUILD_PARALLEL_LEVEL"] = self._cpus(env)
        # Do not leave MSBuild worker nodes running after the build
        env["MSBUILDDISABLENODEREUSE"] = "1"
        return env

    @contextlib.contextmanager
    def _ninja_setup(self, env, cfg):
        """Ninja builds every target of a package in parallel, file by file,
        so no /MP and no /m. The cap on the jobs of all the ninja processes
        together is a jobserver, unless the job sets CMAKE_BUILD_PARALLEL_LEVEL,
        which CMake passes to every ninja as -jN.
        """
        env = dict(env, CMAKE_GENERATOR="Ninja")
        # Set by the vs2019 activation; Ninja accepts no platform or toolset
        for name in ("CMAKE_GENERATOR_PLATFORM", "CMAKE_GENERATOR_TOOLSET"):
            env.pop(name, None)
        # The dependencies build without warnings. CMAKE_CXX_FLAGS=-w, as with
        # MSBuild, would drop the default /EHsc /GR, and gz-cmake adds /W2
        # after it. cl.exe reads _CL_ after its command line, so /w wins
        # (cl.exe prints D9025 "overriding '/W2' with '/w'" for each file).
        deps_env = dict(env, _CL_=" ".join(filter(None, (env.get("_CL_"),
                                                          "/w"))))
        ninja_args = {"cmake_args": (" -GNinja",), "quiet_deps_args": ()}
        if env.get("CMAKE_BUILD_PARALLEL_LEVEL"):
            print("ninja: -j" + env["CMAKE_BUILD_PARALLEL_LEVEL"] +
                  " for every package (CMAKE_BUILD_PARALLEL_LEVEL)",
                  flush=True)
            yield colcon.BuildSetup(deps_env, env, **ninja_args)
            return
        slots = int(self._cpus(env))
        with jobserver.windows_semaphore(jobserver.tokens_for(slots)) as name:
            print(f"ninja: jobserver {name}: up to {slots} jobs for one "
                  "package, one more for each other package building at "
                  "the same time", flush=True)
            flags = jobserver.makeflags(cfg.make_jobs, name)
            yield colcon.BuildSetup(dict(deps_env, MAKEFLAGS=flags),
                                    dict(env, MAKEFLAGS=flags), **ninja_args)

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
