import os
import stat
import sys

import pytest

from pixi_ci.fsops import copy_sources, remove_tree
from pixi_ci.runner import PYTHON, CIError, CommandError, Runner, resolve_executable


def make_tool(directory, name):
    directory.mkdir(parents=True, exist_ok=True)
    tool = directory / name
    tool.write_text("#!/bin/sh\necho from-env\n")
    tool.chmod(0o755)
    return tool


def test_tools_resolve_against_the_activated_path_only(tmp_path):
    tool = make_tool(tmp_path / "env" / "bin", "colcon")
    env = {"PATH": str(tmp_path / "env" / "bin")}
    assert resolve_executable("colcon", env) == str(tool)
    # sh is on the PATH of this process, not on the activated one
    with pytest.raises(CIError, match="sh not found in PATH: .*env"):
        resolve_executable("sh", env)


def test_python_and_absolute_tools_are_not_searched(tmp_path):
    assert resolve_executable(PYTHON, {}) == sys.executable
    assert resolve_executable(tmp_path / "pixi.exe", {}) == str(tmp_path / "pixi.exe")


@pytest.mark.skipif(sys.platform == "win32", reason="uses a shell script")
def test_runner_runs_the_resolved_tool_and_checks_the_exit_code(tmp_path):
    make_tool(tmp_path / "bin", "colcon")
    env = {"PATH": str(tmp_path / "bin")}
    result = Runner().run("colcon", [], cwd=tmp_path, env=env, capture=True)
    assert result.stdout == "from-env\n"
    with pytest.raises(CommandError, match="exit code 3"):
        Runner().run(PYTHON, ["-c", "raise SystemExit(3)"], cwd=tmp_path, env={})
    assert Runner().run(PYTHON, ["-c", "raise SystemExit(3)"], cwd=tmp_path,
                        env={}, check=False).returncode == 3


@pytest.mark.skipif(sys.platform == "win32", reason="POSIX permissions")
def test_remove_tree_removes_read_only_entries(tmp_path):
    objects = tmp_path / "repo" / ".git" / "objects"
    objects.mkdir(parents=True)
    (objects / "pack").write_text("x")
    (objects / "pack").chmod(stat.S_IREAD)
    objects.chmod(stat.S_IREAD | stat.S_IEXEC)   # unlink fails here on POSIX
    remove_tree(tmp_path / "repo")
    assert not (tmp_path / "repo").exists()
    remove_tree(tmp_path / "repo")               # missing: no-op


def test_copy_sources_skips_excluded_entries(tmp_path):
    source = tmp_path / "gz-math"
    (source / ".git").mkdir(parents=True)
    (source / ".git" / "HEAD").write_text("ref")
    (source / "src").mkdir()
    (source / "src" / "Vector3.cc").write_text("code")
    (source / "src" / ".hidden_file").write_text("x")
    (source / "empty").mkdir()
    copy_sources(source, tmp_path / "copy", lambda p: p.name.startswith("."))
    copied = sorted(str(p.relative_to(tmp_path / "copy"))
                    for p in (tmp_path / "copy").rglob("*"))
    assert copied == ["empty", "src", os.path.join("src", "Vector3.cc")]
