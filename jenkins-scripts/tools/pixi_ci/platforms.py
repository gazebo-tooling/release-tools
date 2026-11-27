"""Per-OS facts: Windows on amd64 and macOS on arm64."""

import ntpath
import os
import platform
import stat
import sys
from pathlib import Path

from . import envvars, leakcheck
from .runner import CIError, resolve_executable, section

# macOS: Homebrew stays installed on the agents, other jobs use it
HOMEBREW_ROOTS = ("/opt/homebrew", "/usr/local")
UNSET_PREFIXES = ("HOMEBREW_", "DYLD_")
UNSET_NAMES = frozenset((
    "CMAKE_PREFIX_PATH", "CMAKE_INCLUDE_PATH", "CMAKE_LIBRARY_PATH",
    "CMAKE_PROGRAM_PATH", "CMAKE_FRAMEWORK_PATH", "PKG_CONFIG_PATH",
    "PKG_CONFIG_LIBDIR", "CPATH", "C_INCLUDE_PATH", "CPLUS_INCLUDE_PATH",
    "LIBRARY_PATH", "PYTHONPATH", "PYTHONHOME",
    # The clang activation keeps a pre-set CC, CXX and CXXFLAGS and appends
    # to CFLAGS, CPPFLAGS and LDFLAGS
    "CC", "CXX", "CPP", "OBJC", "CFLAGS", "CXXFLAGS", "CPPFLAGS", "LDFLAGS"))
# The conda-forge packages whose activation sets each compiler variable
CONDA_COMPILERS = {"CC": "clang_osx-arm64", "CXX": "clangxx_osx-arm64"}


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
    cmake_args = ()                # appended to both colcon builds
    # after the console handler of colcon build and test; colcon-notification
    # is in the win-64 envs only
    colcon_event_handlers = ("desktop_notification-",)

    def isolate_env(self, env):
        """The job environment every later step runs with."""
        return env

    def report_toolchain(self, runner, cfg, env):
        """Log the compiler and SDK after the activation: nothing on Windows."""

    def dependency_cxx_flags(self, env):
        """CMAKE_CXX_FLAGS of the build of the dependencies."""
        return "-w"

    def check_build(self, runner, cfg, env):
        """Check the build before the tests: nothing on Windows."""

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


class MacOSPlatform:
    detect_os = "darwin"
    detect_arch = "arm64"
    default_build_type = "RelWithDebInfo"   # as the Homebrew jobs
    hooks_file = "hooks.sh"
    hook_shell = "bash"
    # The find_*() calls of CMake >= 3.23 skip these prefixes, and look in
    # the env before the SDK frameworks and app bundles, as conda-forge
    # builds its macOS packages: with the default FIRST, FindRuby takes the
    # headers of the SDK's Ruby.framework for the env's ruby
    cmake_args = ("-DCMAKE_IGNORE_PREFIX_PATH=" + ";".join(HOMEBREW_ROOTS),
                  "-DCMAKE_FIND_FRAMEWORK=LAST", "-DCMAKE_FIND_APPBUNDLE=LAST")
    # colcon refuses a handler it does not know, even to disable it, and the
    # osx-arm64 envs have no colcon-notification
    colcon_event_handlers = ()

    def isolate_env(self, env):
        """env without Homebrew: its PATH entries, the variables that steer
        compilers and package lookups, and every value that mentions it."""
        isolated, unset, dropped = {}, [], []
        for name, value in env.items():
            if name == "PATH":
                kept = []
                for entry in value.split(":"):
                    (dropped if leakcheck.is_inside(entry, HOMEBREW_ROOTS)
                     else kept).append(entry)
                isolated[name] = ":".join(kept)
            elif (name.startswith(UNSET_PREFIXES) or name in UNSET_NAMES or
                  any(root in value for root in HOMEBREW_ROOTS)):
                unset.append(name)
            else:
                isolated[name] = value
        with section("macOS: remove Homebrew from the job environment"):
            # The names only: values can hold credentials
            print(f"removed from PATH: {' '.join(dropped) or 'nothing'}",
                  flush=True)
            print(f"unset: {' '.join(sorted(unset)) or 'nothing'}", flush=True)
        return isolated

    def report_toolchain(self, runner, cfg, env):
        with section("macOS toolchain"):
            for tool, args in (("sw_vers", []),
                               ("xcrun", ["--show-sdk-path"]),
                               ("xcrun", ["--show-sdk-version"]),
                               (env["CXX"], ["--version"])):
                runner.run(tool, args, cwd=cfg.workspace, env=env)
            print(f"SDKROOT={env['SDKROOT']}", flush=True)

    def dependency_cxx_flags(self, env):
        # -stdlib=libc++ and -isystem $CONDA_PREFIX/include from the activation
        return " ".join(filter(None, [env.get("CXXFLAGS", ""), "-w"]))

    def check_build(self, runner, cfg, env):
        with section("macOS: check the build for Homebrew and system leaks"):
            developer_dir = runner.run("xcode-select", ["-p"],
                                       cwd=cfg.workspace, env=env,
                                       capture=True).stdout.strip()
            leaks = leakcheck.find_leaks(
                runner, cfg.ws, conda_prefix=env["CONDA_PREFIX"],
                sdkroot=env["SDKROOT"], developer_dir=developer_dir, env=env)
            for leak in leaks:
                print(leak, flush=True)
            if leaks:
                raise CIError(f"{len(leaks)} Homebrew or system paths in the "
                              "build, see the list above")
            print("no Homebrew or system paths in the build", flush=True)

    def normalize_key(self, name):
        return name

    def normalize_env(self, environ):
        return dict(environ)

    def references(self, value):
        # pixi expands the POSIX activation values itself
        return set()

    def expand(self, value, env):
        return value

    def _pixi_root(self, env):
        if not env.get("WORKSPACE"):
            raise CIError("WORKSPACE is not set")
        return Path(env["WORKSPACE"]) / ".pixi_ci"

    def default_pixi_exe(self, env):
        return self._pixi_root(env) / "pixi"

    def default_project_path(self, env):
        return self._pixi_root(env) / "project"

    def post_activation(self, env):
        """env, once the activation gave the conda compilers and an SDK."""
        prefix = env.get("CONDA_PREFIX")
        if not prefix:
            raise CIError("CONDA_PREFIX is not set after the pixi activation")
        for name, package in CONDA_COMPILERS.items():
            fix = (f"the env needs {package} under "
                   "[target.osx-arm64.build-dependencies] in its "
                   "conda/envs/<env>/pixi.toml")
            if not env.get(name):
                raise CIError(f"{name} is not set after the pixi activation: "
                              f"{fix}")
            try:
                path = resolve_executable(env[name], env)
            except CIError:
                raise CIError(f"{name}={env[name]} is not in the activated "
                              f"PATH: {fix}") from None
            if not leakcheck.is_inside(path, [prefix]):
                raise CIError(f"{name}={env[name]} is {path}, outside "
                              f"CONDA_PREFIX={prefix}: {fix}")
        sdkroot = env.get("SDKROOT")
        if not sdkroot:
            raise CIError("SDKROOT is not set after the pixi activation: the "
                          "clang activation takes it from xcrun, which needs "
                          "the Xcode Command Line Tools (xcode-select --install)")
        if not Path(sdkroot).is_dir():
            raise CIError(f"SDKROOT={sdkroot} does not exist: install or "
                          "update the Xcode Command Line Tools "
                          "(xcode-select --install)")
        return env

    def is_hidden_or_system(self, path):
        # What xcopy leaves out on Windows: there, Git for Windows hides .git
        return Path(path).name == ".git"

    def check_gpu(self, runner, cfg, env):
        # Apple GPUs are always there and no tool reports them like dxdiag
        print("GPU_SUPPORT_NEEDED: nothing to check on macOS", flush=True)


def current():
    if sys.platform == "win32":
        return WindowsPlatform()
    if sys.platform == "darwin":
        if platform.machine() == "arm64":
            return MacOSPlatform()
        raise CIError("pixi_ci supports macOS on arm64 only")
    raise CIError(f"pixi_ci does not support {sys.platform} yet")
