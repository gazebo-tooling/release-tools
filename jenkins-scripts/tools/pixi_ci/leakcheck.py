"""macOS: find Homebrew and other non-conda paths in a colcon build.

Homebrew stays installed on the macOS agents, so a build can pick a library
or a tool from it, or from any other prefix outside the conda env (/opt/X11,
/Library/Frameworks, MacPorts). The check is an allowlist, so a prefix nobody
thought of is refused too. A wrong refusal is fixed by extending the list.
"""

import os
import posixpath
import re
from pathlib import Path, PurePosixPath

# Load commands that name a library to load, and the one that adds an rpath
DYLIB_COMMANDS = {"LC_LOAD_DYLIB", "LC_LOAD_WEAK_DYLIB", "LC_REEXPORT_DYLIB",
                  "LC_LAZY_LOAD_DYLIB", "LC_LOAD_UPWARD_DYLIB"}
RPATH_COMMAND = "LC_RPATH"
RELATIVE_LINK_PREFIXES = ("@rpath/", "@loader_path", "@executable_path")
# The system C++ runtime next to conda's @rpath/libc++.1.dylib: the build did
# not use conda clang. /usr/lib/libc++abi.dylib is fine: conda-forge ships no
# libc++abi, its libc++ uses the system one.
REFUSED_LINKS = {"/usr/lib/libc++.1.dylib"}
SYSTEM_LIBRARY_ROOTS = ("/usr/lib", "/System/Library")
SYSTEM_TOOL_ROOTS = ("/usr/bin", "/bin", "/usr/sbin", "/sbin")
COMPILER_ENTRIES = {"CMAKE_C_COMPILER", "CMAKE_CXX_COMPILER"}
PATH_TYPES = {"PATH", "FILEPATH"}

_THIN_MAGICS = {b"\xcf\xfa\xed\xfe", b"\xce\xfa\xed\xfe",
                b"\xfe\xed\xfa\xcf", b"\xfe\xed\xfa\xce"}
_FAT_MAGICS = {b"\xca\xfe\xba\xbe", b"\xca\xfe\xba\xbf"}
_COMMAND = re.compile(r"^\s*cmd (\S+)\s*$")
_VALUE = re.compile(r"^\s*(name|path) (.*) \(offset \d+\)\s*$")


def is_macho(path):
    """A regular file (not a symlink) with a thin or fat Mach-O header.

    Java class files share the fat magic: a fat header is followed by the
    number of architectures, a class file by its version (45 or more).
    """
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        return False
    with path.open("rb") as file:
        header = file.read(8)
    if header[:4] in _THIN_MAGICS:
        return True
    return (header[:4] in _FAT_MAGICS and len(header) == 8 and
            1 <= int.from_bytes(header[4:8], "big") <= 19)


def load_commands(otool_output):
    """(command, path) of the dylib load commands and rpaths in otool -l."""
    found = []
    command = None
    for line in otool_output.splitlines():
        match = _COMMAND.match(line)
        if match:
            command = match.group(1)
            continue
        match = _VALUE.match(line)
        if match and (command in DYLIB_COMMANDS and match.group(1) == "name" or
                      command == RPATH_COMMAND and match.group(1) == "path"):
            found.append((command, match.group(2)))
    return found


def cache_entries(text):
    """(name, type, value) of the NAME:TYPE=VALUE lines of a CMakeCache.txt."""
    entries = []
    for line in text.splitlines():
        if not line.strip() or line.startswith(("//", "#")):
            continue
        key, equal, value = line.partition("=")
        name, colon, kind = key.rpartition(":")
        if equal and colon:
            entries.append((name, kind, value))
    return entries


def is_inside(path, roots):
    """path is one of roots or below it, component by component, after
    resolving `..`: /env-old is not inside /env. Empty roots are ignored."""
    path = PurePosixPath(posixpath.normpath(path))
    return any(path.is_relative_to(posixpath.normpath(root))
               for root in roots if root)


def _link_allowed(name, roots):
    if posixpath.normpath(name) in REFUSED_LINKS:
        return False
    return name.startswith(RELATIVE_LINK_PREFIXES) or is_inside(name, roots)


def _with_physical(roots):
    """roots, then their physical paths. CMake and colcon record the physical
    path of their working directory, so with WORKSPACE behind a symlink (/tmp
    is one on macOS) the build tree shows up resolved; so can the env, and
    the SDK path of xcrun is a symlink."""
    return tuple(roots) + tuple(Path(os.path.realpath(root)).as_posix()
                                for root in roots if root)


def _install_files(install):
    for directory, _, names in os.walk(install):
        for name in names:
            yield Path(directory, name)


def find_leaks(runner, ws, *, conda_prefix, sdkroot, developer_dir, env):
    """Lines '<file relative to ws>: <load command or cache entry> <path>'
    for every path outside the allowlists, sorted."""
    ws = Path(ws)
    compiler_roots = _with_physical((conda_prefix,))
    link_roots = (_with_physical((conda_prefix, ws.as_posix()))
                  + SYSTEM_LIBRARY_ROOTS)
    cache_roots = (_with_physical((conda_prefix, ws.as_posix(), sdkroot,
                                   developer_dir))
                   + SYSTEM_TOOL_ROOTS + SYSTEM_LIBRARY_ROOTS)
    leaks = set()
    for path in sorted(p for p in _install_files(ws / "install") if is_macho(p)):
        output = runner.run("otool", ["-l", path], cwd=ws, env=env,
                            capture=True).stdout
        for command, name in load_commands(output):
            if not _link_allowed(name, link_roots):
                leaks.add(f"{path.relative_to(ws).as_posix()}: {command} {name}")
    for cache in sorted(ws.glob("build/*/CMakeCache.txt")):
        text = cache.read_text(encoding="utf-8", errors="replace")
        for name, kind, value in cache_entries(text):
            if name in COMPILER_ENTRIES:
                roots = compiler_roots
            elif kind in PATH_TYPES:
                roots = cache_roots
            else:
                continue
            for item in value.split(";"):
                # Empty and relative values name no prefix, and neither does
                # the <VAR>-NOTFOUND of a find_*() that found nothing
                if not posixpath.isabs(item):
                    continue
                if not is_inside(item, roots):
                    leaks.add(f"{cache.relative_to(ws).as_posix()}: {name} {item}")
    return sorted(leaks)
