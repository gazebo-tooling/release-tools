"""Subprocess execution and Jenkins log sections."""

import contextlib
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass

# Tool name of the interpreter running pixi_ci (the bootstrap env python,
# which has pyyaml). Used to run the release-tools helper scripts.
PYTHON = "{python}"


class CIError(RuntimeError):
    """A job failure, reported as one ERROR line in the Jenkins log."""


class CommandError(CIError):
    def __init__(self, command, returncode):
        super().__init__(f"exit code {returncode}: {command}")
        self.returncode = returncode


@dataclass
class Result:
    returncode: int
    stdout: str = ""


def resolve_executable(tool, env):
    """Absolute path of tool, searched in the PATH of env only.

    On Windows CreateProcess searches the PATH of the parent process, not
    the one in env, so a bare name could run the wrong binary.
    """
    tool = str(tool)
    if tool == PYTHON:
        return sys.executable
    if os.path.isabs(tool):
        return tool
    search_path = env.get("PATH", "")
    found = shutil.which(tool, path=search_path)
    if found is None:
        raise CIError(f"{tool} not found in PATH: {search_path}")
    return found


@contextlib.contextmanager
def section(title):
    print(f"# BEGIN SECTION: {title}", flush=True)
    try:
        yield
    finally:
        print("# END SECTION", flush=True)


class Runner:
    def run(self, tool, args, *, cwd, env, check=True, capture=False):
        """Run tool with args; capture=True returns its stdout as text."""
        command = [resolve_executable(tool, env)] + [str(a) for a in args]
        print(f"+ {subprocess.list2cmdline(command)}", flush=True)
        completed = subprocess.run(
            command, cwd=cwd, env=env,
            stdout=subprocess.PIPE if capture else None,
            encoding="utf-8" if capture else None,
            errors="replace" if capture else None)
        if check and completed.returncode != 0:
            raise CommandError(subprocess.list2cmdline(command),
                               completed.returncode)
        return Result(completed.returncode, completed.stdout or "")
