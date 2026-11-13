"""File operations that behave like the cmd built-ins they replace."""

import os
import shutil
import stat
import sys
from pathlib import Path


def _make_writable_and_retry(function, path, _error):
    # git marks objects and packs read-only: rmdir /s /q removes them anyway
    for target in (path, os.path.dirname(path)):
        os.chmod(target, os.stat(target).st_mode | stat.S_IWRITE)
    function(path)


def remove_tree(path):
    """rmdir /s /q: remove the tree, read-only files included; no-op if missing."""
    path = Path(path)
    if not path.exists():
        return
    if sys.version_info >= (3, 12):
        shutil.rmtree(path, onexc=_make_writable_and_retry)
    else:
        shutil.rmtree(path, onerror=_make_writable_and_retry)


def copy_sources(source, destination, is_excluded):
    """xcopy /s /e /i: copy the tree, leaving out the entries is_excluded() rejects."""
    def ignore(directory, names):
        return {name for name in names if is_excluded(Path(directory) / name)}
    shutil.copytree(source, destination, ignore=ignore)
