"""The commands the driver runs for a macOS job: the Windows sequence plus
the Homebrew isolation, the toolchain report and the leak check."""

import sys

import pytest

from fakes import (CC, CXX, SHELL_HOOKS, commands, find, make_job, make_tool,
                   sections)
from pixi_ci.platforms import MacOSPlatform
from pixi_ci.runner import PYTHON, Result
from pixi_ci.workspace import GAZEBODISTRO_URL

pytestmark = pytest.mark.skipif(
    sys.platform == "win32", reason="the compilers are found in a POSIX PATH")

DEVELOPER_DIR = "/Library/Developer/CommandLineTools"
IGNORE_HOMEBREW = "-DCMAKE_IGNORE_PREFIX_PATH=/opt/homebrew;/usr/local"
# What a Jenkins agent with Homebrew can pass to the job
HOMEBREW_ENV = {
    "PATH": "/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin",
    "HOMEBREW_PREFIX": "/opt/homebrew",
    "DYLD_LIBRARY_PATH": "/opt/homebrew/lib",
    "CXX": "clang++",
    "CXXFLAGS": "-I/opt/homebrew/include",
    "CMAKE_PREFIX_PATH": "/opt/homebrew",
    "PKG_CONFIG_PATH": "/opt/homebrew/lib/pkgconfig",
    "AR": "/opt/homebrew/bin/ar",
    "JAVA_HOME": "/usr/local/opt/openjdk",
}
THIN_MACHO = b"\xcf\xfa\xed\xfe" + bytes(28)


def on_call(job, predicate, action):
    """Run action(call) before the job answers each call matching predicate."""
    respond = job.respond

    def wrapped(call):
        if predicate(call):
            action(call)
        return respond(call)
    job.respond = wrapped


def is_build(call):
    return call.tool == "colcon" and call.args[:1] == ["build"]


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def macos_job(tmp_path, **kwargs):
    """A job on an agent with Homebrew; returns (job, CONDA_PREFIX)."""
    job = make_job(tmp_path, platform=MacOSPlatform(), **kwargs)
    job.environ = dict(HOMEBREW_ENV, WORKSPACE=str(job.workspace),
                       MAKE_JOBS="4")
    prefix = job.project / ".pixi" / "envs" / "default"
    sdk = tmp_path / "MacOSX.sdk"
    sdk.mkdir()
    job.activation = {
        "CONDA_PREFIX": str(prefix),
        "PATH": f"{prefix}/bin:/usr/bin:/bin",
        "CC": CC,
        "CXX": CXX,
        "CXXFLAGS": f"-stdlib=libc++ -isystem {prefix}/include",
        "LDFLAGS": f"-Wl,-rpath,{prefix}/lib -L{prefix}/lib",
        "SDKROOT": str(sdk),
    }
    answers = {("sw_vers",): "ProductName:\tmacOS\nProductVersion:\t15.5\n",
               ("xcrun", "--show-sdk-path"): f"{sdk}\n",
               ("xcrun", "--show-sdk-version"): "15.5\n",
               (CXX, "--version"): "clang version 21.1.0\n",
               ("xcode-select", "-p"): f"{DEVELOPER_DIR}\n"}
    job.replies += [(lambda call, key=key: (call.tool, *call.args) == key,
                     Result(0, text)) for key, text in answers.items()]

    def install(call):
        # pixi install creates the env, with the compilers of the activation
        for compiler in (CC, CXX):
            make_tool(prefix / "bin" / compiler)
    on_call(job, lambda c: c.tool == job.pixi and c.args[:1] == ["install"],
            install)
    return job, prefix


def test_gz_math_jetty_on_macos(tmp_path, capsys):
    job, prefix = macos_job(tmp_path)
    code, calls = job.run()
    rt, ws_root, ws = job.rt, job.workspace, job.workspace / "ws"
    tools, dsl = rt / "jenkins-scripts" / "tools", rt / "jenkins-scripts" / "dsl"
    build = ["build", "--build-base", "build", "--install-base", "install",
             "--parallel-workers", "4"]
    build_type = ["--cmake-args", " -DCMAKE_BUILD_TYPE=RelWithDebInfo"]
    # no desktop_notification-: the osx-arm64 envs have no colcon-notification
    # and colcon refuses a handler it does not know, even to disable it
    handlers = ["--event-handler", "console_cohesion+"]
    assert code == 0
    assert commands(calls) == [
        (PYTHON, [str(tools / "detect_cmake_major_version.py"),
                  str(ws_root / "gz-math" / "CMakeLists.txt")], ws_root),
        (PYTHON, [str(dsl / "tools" / "get_ciconfigs_from_package_and_version.py"),
                  "--conda-env", "--os", "darwin", "--arch", "arm64",
                  "--yaml-file", str(dsl / "gz-collections.yaml"),
                  "gz-math", "9"], ws_root),
        (job.pixi, ["install", "--locked"], job.project),
        (job.pixi, ["info"], job.project),
        (job.pixi, ["list"], job.project),
        (job.pixi, ["shell-hook", "--locked", "--shell", "bash"], job.project),
        (job.pixi, ["shell-hook", "--json", "--locked"], job.project),
        # the toolchain the activation gave
        ("sw_vers", [], ws_root),
        ("xcrun", ["--show-sdk-path"], ws_root),
        ("xcrun", ["--show-sdk-version"], ws_root),
        (CXX, ["--version"], ws_root),
        # as on Windows
        ("git", ["clone", GAZEBODISTRO_URL, str(ws_root / "gazebodistro"),
                 "-b", "master"], ws_root),
        ("vcs", ["import", "--retry", "5", "--force", "--input",
                 str(ws_root / "gazebodistro" / "gz-math9.yaml"),
                 str(ws / "src")], ws_root),
        ("vcs", ["pull"], ws / "src"),
        ("colcon", ["list", "-t"], ws),
        ("vcs", ["export", "--exact"], ws),
        ("colcon", ["list", "--names-only"], ws),
        ("colcon", build + ["--packages-skip", "gz-math"] + build_type
         + ["-DBUILD_TESTING=0",
            f"-DCMAKE_CXX_FLAGS=-stdlib=libc++ -isystem {prefix}/include -w",
            IGNORE_HOMEBREW] + handlers, ws),
        ("colcon", build + ["--packages-select", "gz-math"] + build_type
         + [IGNORE_HOMEBREW, " -DBUILD_TESTING=1"] + handlers, ws),
        # the leak check; install/ has no Mach-O file for otool here
        ("xcode-select", ["-p"], ws_root),
        ("colcon", ["test", "--install-base", "install",
                    "--packages-select", "gz-math", "--executor", "sequential",
                    "--event-handler", "console_direct+"], ws),
        ("colcon", ["test-result", "--all"], ws),
    ]
    assert sections(capsys) == [
        "pixi_ci configuration",
        "macOS: remove Homebrew from the job environment",
        "auto-detect conda environment",
        "pixi: create noble_like environment",
        "pixi: info",
        "pixi: list packages",
        "pixi: enable shell",
        "pixi: custom environment variable for gz",
        "macOS toolchain",
        "setup workspace",
        "get open robotics deps (gz-math9.yaml) sources into the workspace",
        "move gz-math source to workspace",
        "packages in workspace",
        "Check if package gz-math9 is in colcon workspace",
        "compiling gz-math",
        "colcon compilation without test for dependencies of gz-math",
        "colcon compilation with tests for gz-math",
        "macOS: check the build for Homebrew and system leaks",
        "running tests for gz-math",
        "colcon test for gz-math",
        "colcon test-result",
        "export testing results",
        "clean up workspace",
    ]
    assert (job.project / "hooks.sh").read_text() == SHELL_HOOKS["bash"]
    assert (ws_root / "build" / "test_results" / "gz-math" /
            "UNIT_Vector3_TEST.xml").is_file()
    assert not ws.exists()


def test_no_command_sees_homebrew(tmp_path):
    job, prefix = macos_job(tmp_path)
    code, calls = job.run()
    assert code == 0
    for call in calls:
        assert not ({"HOMEBREW_PREFIX", "CMAKE_PREFIX_PATH",
                     "DYLD_LIBRARY_PATH"} & call.env.keys())
        assert not any(root in value for value in call.env.values()
                       for root in ("/opt/homebrew", "/usr/local"))
    # pixi and the helper scripts run in the cleaned job environment
    assert all(c.env["PATH"] == "/usr/bin:/bin" for c in calls
               if c.tool in (PYTHON, job.pixi))
    build_env = next(c.env for c in calls if is_build(c))
    assert build_env["CC"] == CC
    assert build_env["CXX"] == CXX
    assert build_env["PATH"] == f"{prefix}/bin:/usr/bin:/bin"
    assert build_env["MAKEFLAGS"] == "-j4"


def test_homebrew_in_cmake_cache_fails_before_tests(tmp_path, capsys):
    job, _ = macos_job(tmp_path)
    on_call(job, is_build, lambda call: write(
        call.cwd / "build" / "gz-math" / "CMakeCache.txt",
        b"//Eigen3 directory\nEigen3_DIR:PATH=/opt/homebrew/share/eigen3/cmake\n"))
    code, calls = job.run()
    output = capsys.readouterr().out
    assert code == 1
    assert ("build/gz-math/CMakeCache.txt: Eigen3_DIR "
            "/opt/homebrew/share/eigen3/cmake") in output
    assert ("ERROR: 1 Homebrew or system paths in the build, see the list "
            "above") in output
    assert calls[-1].tool == "xcode-select"
    assert not find(calls, "colcon", "test")


def test_homebrew_linked_library_fails_before_tests(tmp_path, capsys):
    job, _ = macos_job(tmp_path)
    library = job.workspace / "ws" / "install" / "lib" / "libgz-math.dylib"
    on_call(job, is_build, lambda call: write(library, THIN_MACHO))
    job.replies.append((lambda call: call.tool == "otool", Result(0, (
        f"{library}:\nLoad command 12\n          cmd LC_LOAD_DYLIB\n"
        "      cmdsize 72\n"
        "         name /opt/homebrew/opt/eigen/lib/libx.dylib (offset 24)\n"))))
    code, calls = job.run()
    output = capsys.readouterr().out
    assert code == 1
    assert ("install/lib/libgz-math.dylib: LC_LOAD_DYLIB "
            "/opt/homebrew/opt/eigen/lib/libx.dylib") in output
    assert [c.args for c in find(calls, "otool")] == [["-l", str(library)]]
    assert not find(calls, "colcon", "test")


def test_gpu_support_needed_is_a_no_op(tmp_path, capsys):
    job, _ = macos_job(tmp_path)
    code, calls = job.run(GPU_SUPPORT_NEEDED="true")
    assert code == 0
    assert not find(calls, "dxdiag")
    assert ("GPU_SUPPORT_NEEDED: nothing to check on macOS"
            in capsys.readouterr().out)
