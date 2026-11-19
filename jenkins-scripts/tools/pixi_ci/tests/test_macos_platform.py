"""MacOSPlatform: the Homebrew isolation, the checks after the activation
and the defaults."""

import platform
import sys
from pathlib import Path

import pytest

from fakes import CC, CXX, make_tool
from pixi_ci import envvars, platforms
from pixi_ci.platforms import MacOSPlatform, WindowsPlatform
from pixi_ci.runner import CIError

POSIX_ONLY = pytest.mark.skipif(sys.platform == "win32",
                                reason="resolves extensionless executables")


@pytest.fixture
def activated(tmp_path):
    """An activated env whose compilers and SDK exist."""
    prefix = tmp_path / "env"
    make_tool(prefix / "bin" / CC)
    make_tool(prefix / "bin" / CXX)
    make_tool(tmp_path / "usr" / "bin" / "clang++")
    (tmp_path / "MacOSX.sdk").mkdir()
    return {"CONDA_PREFIX": str(prefix),
            "PATH": f"{prefix / 'bin'}:{tmp_path / 'usr' / 'bin'}",
            "CC": CC, "CXX": CXX, "SDKROOT": str(tmp_path / "MacOSX.sdk")}


def test_isolation_removes_homebrew_and_compiler_settings(capsys):
    env = {"PATH": "/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin",
           "HOMEBREW_PREFIX": "/opt/homebrew",
           "DYLD_LIBRARY_PATH": "/opt/homebrew/lib",
           "CXX": "clang++",
           "CXXFLAGS": "-I/opt/homebrew/include",
           "CMAKE_PREFIX_PATH": "/opt/homebrew",
           "PKG_CONFIG_PATH": "/opt/homebrew/lib/pkgconfig",
           "AR": "/opt/homebrew/bin/ar",
           "JAVA_HOME": "/usr/local/opt/openjdk",
           "HOME": "/Users/jenkins",
           "LANG": "en_US.UTF-8",
           "WORKSPACE": "/Users/jenkins/workspace/gz_math-9-cnlmac",
           "SECRET_TOKEN": "s3cr3t"}
    original = dict(env)
    isolated = MacOSPlatform().isolate_env(env)
    out = capsys.readouterr().out
    assert isolated == {"PATH": "/usr/bin:/bin",
                        "HOME": "/Users/jenkins",
                        "LANG": "en_US.UTF-8",
                        "WORKSPACE": "/Users/jenkins/workspace/gz_math-9-cnlmac",
                        "SECRET_TOKEN": "s3cr3t"}
    assert env == original
    assert "# BEGIN SECTION: macOS: remove Homebrew from the job environment" in out
    assert "/opt/homebrew/bin" in out and "/usr/local/bin" in out
    for name in ("AR", "CMAKE_PREFIX_PATH", "CXX", "CXXFLAGS",
                 "DYLD_LIBRARY_PATH", "HOMEBREW_PREFIX", "JAVA_HOME",
                 "PKG_CONFIG_PATH"):
        assert name in out
    for value in ("s3cr3t", "/usr/local/opt/openjdk", "-I/opt/homebrew/include",
                  "/opt/homebrew/bin/ar", "/opt/homebrew/lib/pkgconfig",
                  "clang++"):
        assert value not in out


def test_isolation_keeps_prefixes_that_only_start_alike(capsys):
    env = {"PATH": "/usr/localbin:/opt/homebrew-tools/bin:/usr/bin"}
    assert MacOSPlatform().isolate_env(env) == env


@POSIX_ONLY
def test_post_activation_requires_conda_compilers(activated):
    platform_ = MacOSPlatform()
    assert platform_.post_activation(activated) == activated
    without_cxx = {k: v for k, v in activated.items() if k != "CXX"}
    for broken in (without_cxx, dict(activated, CXX="clang++"),
                   dict(activated, CXX="/opt/homebrew/bin/clang++")):
        with pytest.raises(CIError, match="clangxx_osx-arm64"):
            platform_.post_activation(broken)
    with pytest.raises(CIError, match="clang_osx-arm64"):
        platform_.post_activation(dict(activated, CC="clang"))
    with pytest.raises(CIError, match="CONDA_PREFIX is not set"):
        platform_.post_activation(
            {k: v for k, v in activated.items() if k != "CONDA_PREFIX"})


@POSIX_ONLY
def test_missing_sdkroot_fails_naming_the_command_line_tools(activated,
                                                             tmp_path):
    without = {k: v for k, v in activated.items() if k != "SDKROOT"}
    missing = dict(activated, SDKROOT=str(tmp_path / "MacOSX99.sdk"))
    for broken in (without, missing):
        with pytest.raises(CIError, match="Command Line Tools") as error:
            MacOSPlatform().post_activation(broken)
        assert "xcode-select --install" in str(error.value)


def test_dependency_cxx_flags_appends_w():
    macos = MacOSPlatform()
    assert macos.dependency_cxx_flags({"CXXFLAGS": "-stdlib=libc++"}) == \
        "-stdlib=libc++ -w"
    assert macos.dependency_cxx_flags({}) == "-w"


def test_defaults_live_under_the_workspace():
    macos = MacOSPlatform()
    env = {"WORKSPACE": "/Users/jenkins/workspace/job"}
    assert macos.default_pixi_exe(env) == \
        Path("/Users/jenkins/workspace/job/.pixi_ci/pixi")
    assert macos.default_project_path(env) == \
        Path("/Users/jenkins/workspace/job/.pixi_ci/project")
    for unset in ({}, {"WORKSPACE": ""}):
        with pytest.raises(CIError, match="WORKSPACE is not set"):
            macos.default_pixi_exe(unset)


def test_current_accepts_only_arm64_macos(monkeypatch):
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setattr(platform, "machine", lambda: "arm64")
    assert isinstance(platforms.current(), MacOSPlatform)
    monkeypatch.setattr(platform, "machine", lambda: "x86_64")
    with pytest.raises(CIError, match="pixi_ci supports macOS on arm64 only"):
        platforms.current()
    monkeypatch.setattr(sys, "platform", "win32")
    assert isinstance(platforms.current(), WindowsPlatform)
    monkeypatch.setattr(sys, "platform", "linux")
    with pytest.raises(CIError, match="pixi_ci does not support linux"):
        platforms.current()


def test_activation_values_are_used_verbatim():
    base = {"PATH": "/usr/bin:/bin", "HOME": "/Users/jenkins"}
    merged = envvars.merge_activation(
        base, {"PATH": "/env/bin:/usr/bin:/bin", "FOO": "$HOME/x"},
        MacOSPlatform())
    assert merged == {"PATH": "/env/bin:/usr/bin:/bin",
                      "HOME": "/Users/jenkins", "FOO": "$HOME/x"}


def test_only_git_is_left_out_of_the_sources_copy():
    macos = MacOSPlatform()
    assert macos.is_hidden_or_system(Path("gz-math") / ".git")
    assert not macos.is_hidden_or_system(Path("gz-math") / ".github")
    assert not macos.is_hidden_or_system(Path("gz-math") / ".clang-format")


def test_gpu_check_has_nothing_to_check(capsys):
    MacOSPlatform().check_gpu(None, None, {})
    assert "GPU_SUPPORT_NEEDED: nothing to check on macOS" in \
        capsys.readouterr().out
