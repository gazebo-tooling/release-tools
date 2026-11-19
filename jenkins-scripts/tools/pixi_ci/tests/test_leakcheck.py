"""The macOS leak check: Mach-O detection, the otool and CMakeCache parsers
and the allowlists applied to an install tree and its CMake caches."""

import pytest

from fakes import FakeRunner
from pixi_ci.leakcheck import (cache_entries, find_leaks, is_inside, is_macho,
                               load_commands)
from pixi_ci.runner import Result

CONDA = "/w/.pixi_ci/project/.pixi/envs/default"
SDKROOT = "/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk"
DEVELOPER_DIR = "/Library/Developer/CommandLineTools"
THIN = b"\xcf\xfa\xed\xfe" + bytes(28)


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


def symlink_or_skip(link, target):
    try:
        link.symlink_to(target)
    except OSError:
        pytest.skip("creating symlinks needs a privilege here")


def otool_output(path, commands):
    """otool -l text with one load command per (cmd, name or path) pair."""
    lines = [f"{path} (architecture arm64):", "Mach header",
             "      magic  cputype cpusubtype  caps    filetype ncmds",
             " 0xfeedfacf 16777228          0  0x00           6    20"]
    for number, (cmd, value) in enumerate(commands):
        field = "path" if cmd == "LC_RPATH" else "name"
        lines += [f"Load command {number}", f"          cmd {cmd}",
                  "      cmdsize 56",
                  f"         {field} {value} (offset 24)",
                  "   time stamp 2 Thu Jan  1 00:00:02 1970"]
    return "\n".join(lines) + "\n"


def leaks(ws, otool=None):
    """find_leaks with otool answering otool[file name]; returns (lines, files)."""
    otool = otool or {}
    runner = FakeRunner(lambda call: Result(0, otool_output(
        call.args[1], otool.get(call.args[1].replace("\\", "/").split("/")[-1], []))))
    found = find_leaks(runner, ws, conda_prefix=CONDA, sdkroot=SDKROOT,
                       developer_dir=DEVELOPER_DIR, env={"PATH": "/usr/bin"})
    assert all(c.tool == "otool" and c.args[0] == "-l" and c.cwd == ws
               for c in runner.calls)
    return found, sorted(c.args[1].replace("\\", "/").split("/")[-1]
                         for c in runner.calls)


def write_cache(ws, package, lines):
    return write(ws / "build" / package / "CMakeCache.txt",
                 ("# This is the CMakeCache file.\n//Path to a file.\n"
                  + "\n".join(lines) + "\n").encode())


def test_macho_detection(tmp_path):
    thin = [write(tmp_path / f"thin{i}", magic + bytes(28)) for i, magic in
            enumerate((b"\xcf\xfa\xed\xfe", b"\xce\xfa\xed\xfe",
                       b"\xfe\xed\xfa\xcf", b"\xfe\xed\xfa\xce"))]
    fat = write(tmp_path / "fat", b"\xca\xfe\xba\xbe" + (2).to_bytes(4, "big")
                + bytes(40))
    java = write(tmp_path / "A.class", b"\xca\xfe\xba\xbe\x00\x00\x00\x34"
                 + bytes(8))
    others = [java,
              write(tmp_path / "libx.a", b"!<arch>\n" + bytes(8)),
              write(tmp_path / "run.sh", b"#!/bin/sh\necho\n"),
              write(tmp_path / "empty", b""),
              write(tmp_path / "short_fat", b"\xca\xfe\xba\xbe"),
              tmp_path]
    assert all(is_macho(path) for path in thin + [fat])
    assert not any(is_macho(path) for path in others)
    symlink_or_skip(tmp_path / "link", thin[0])
    assert not is_macho(tmp_path / "link")


def test_load_commands_reads_the_dylib_family_and_rpaths():
    output = otool_output("/w/ws/install/lib/libgz-math9.dylib", [
        ("LC_ID_DYLIB", "@rpath/libgz-math9.9.dylib"),
        ("LC_LOAD_DYLINKER", "/usr/lib/dyld"),
        ("LC_LOAD_DYLIB", "@rpath/libc++.1.dylib"),
        ("LC_LOAD_WEAK_DYLIB", "/opt/homebrew/lib/libweak.dylib"),
        ("LC_REEXPORT_DYLIB", "/Applications/My App.app/libre.dylib"),
        ("LC_RPATH", "@loader_path/../lib"),
    ])
    assert load_commands(output) == [
        ("LC_LOAD_DYLIB", "@rpath/libc++.1.dylib"),
        ("LC_LOAD_WEAK_DYLIB", "/opt/homebrew/lib/libweak.dylib"),
        ("LC_REEXPORT_DYLIB", "/Applications/My App.app/libre.dylib"),
        ("LC_RPATH", "@loader_path/../lib"),
    ]


def test_cache_entries_split_name_type_value():
    text = ("# This is the CMakeCache file.\n//Flags used by the compiler\n"
            "CMAKE_CXX_FLAGS:STRING=-DA=1\n\nEigen3_DIR:PATH=/x/share\n")
    assert cache_entries(text) == [("CMAKE_CXX_FLAGS", "STRING", "-DA=1"),
                                   ("Eigen3_DIR", "PATH", "/x/share")]


def test_inside_is_component_wise():
    assert not is_inside("/p/envs/default-old/lib", ["/p/envs/default"])
    assert not is_inside("/p/envs/default/../../opt/homebrew",
                         ["/p/envs/default"])
    assert is_inside("/p/envs/default/lib", ["/p/envs/default"])
    assert is_inside("/p/envs/default", ["/p/envs/default/"])
    # An empty root (an unknown SDKROOT) allows nothing
    assert not is_inside("lib", ["", "/usr/lib"])


def test_allowed_links_and_rpaths_pass(tmp_path):
    ws = tmp_path / "ws"
    lib = write(ws / "install" / "lib" / "libgz-math9.9.dylib", THIN)
    symlink_or_skip(ws / "install" / "lib" / "libgz-math9.dylib", lib)
    write(ws / "install" / "share" / "A.class", b"\xca\xfe\xba\xbe\x00\x00\x00\x34")
    write(ws / "install" / "lib" / "libstatic.a", b"!<arch>\n")
    write(ws / "install" / "bin" / "gz-math-config.sh", b"#!/bin/sh\n")
    found, otool_files = leaks(ws, {"libgz-math9.9.dylib": [
        ("LC_LOAD_DYLIB", "@rpath/libgz-utils3.dylib"),
        ("LC_LOAD_DYLIB", "@loader_path/../lib/libx.dylib"),
        ("LC_LOAD_DYLIB", "@executable_path/../lib/liby.dylib"),
        ("LC_LOAD_DYLIB", "/usr/lib/libSystem.B.dylib"),
        # conda-forge ships no libc++abi: its libc++ uses the system one
        ("LC_LOAD_DYLIB", "/usr/lib/libc++abi.dylib"),
        ("LC_LOAD_DYLIB",
         "/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation"),
        ("LC_LOAD_DYLIB", f"{ws.as_posix()}/install/lib/libgz-utils3.dylib"),
        ("LC_RPATH", f"{CONDA}/lib"),
        ("LC_RPATH", "@loader_path/../lib"),
    ]})
    assert found == []
    assert otool_files == ["libgz-math9.9.dylib"]


def test_homebrew_and_system_cxx_runtime_links_are_reported(tmp_path):
    ws = tmp_path / "ws"
    write(ws / "install" / "lib" / "libgz-math9.dylib", THIN)
    found, _ = leaks(ws, {"libgz-math9.dylib": [
        ("LC_LOAD_DYLIB", "/opt/homebrew/opt/eigen/lib/x.dylib"),
        ("LC_RPATH", "/usr/local/lib"),
        ("LC_LOAD_DYLIB", "/usr/lib/libc++.1.dylib"),
        ("LC_LOAD_DYLIB", "@rpath/libc++.1.dylib"),
    ]})
    assert found == [
        "install/lib/libgz-math9.dylib: LC_LOAD_DYLIB "
        "/opt/homebrew/opt/eigen/lib/x.dylib",
        "install/lib/libgz-math9.dylib: LC_LOAD_DYLIB /usr/lib/libc++.1.dylib",
        "install/lib/libgz-math9.dylib: LC_RPATH /usr/local/lib",
    ]


def test_cache_refuses_paths_outside_the_allowlist(tmp_path):
    ws = tmp_path / "ws"
    write_cache(ws, "gz-math", [
        "Eigen3_DIR:PATH=/opt/homebrew/share/eigen3/cmake",
        "X11_X11_LIB:FILEPATH=/opt/X11/lib/libX11.dylib",
        "Foo_DIR:PATH=/Library/Frameworks/Foo.framework",
        "Bar_LIBRARY:FILEPATH=/opt/local/lib/libbar.dylib",
        f"Baz_LIBRARIES:FILEPATH={CONDA}/lib/libbaz.dylib;"
        "/usr/local/lib/libqux.dylib",
        # not reported
        "Missing_LIBRARY:FILEPATH=Missing_LIBRARY-NOTFOUND",
        "CMAKE_INSTALL_LIBDIR:PATH=lib",
        "CMAKE_CXX_FLAGS:STRING=-I/opt/homebrew/include",
        "Empty_DIR:PATH=",
        f"Good_DIR:PATH={CONDA}/lib/cmake/Good",
        f"OPENGL_gl_LIBRARY:FILEPATH={SDKROOT}/System/Library/Frameworks/"
        "OpenGL.framework",
        f"CMAKE_TAPI:FILEPATH={DEVELOPER_DIR}/usr/bin/tapi",
        "GIT_EXECUTABLE:FILEPATH=/usr/bin/git",
        "CMAKE_MAKE_PROGRAM:FILEPATH=/usr/bin/make",
        f"CMAKE_INSTALL_PREFIX:PATH={ws.as_posix()}/install/gz-math",
    ])
    cache = "build/gz-math/CMakeCache.txt"
    assert leaks(ws)[0] == [
        f"{cache}: Bar_LIBRARY /opt/local/lib/libbar.dylib",
        f"{cache}: Baz_LIBRARIES /usr/local/lib/libqux.dylib",
        f"{cache}: Eigen3_DIR /opt/homebrew/share/eigen3/cmake",
        f"{cache}: Foo_DIR /Library/Frameworks/Foo.framework",
        f"{cache}: X11_X11_LIB /opt/X11/lib/libX11.dylib",
    ]


def test_compilers_must_come_from_conda(tmp_path):
    ws = tmp_path / "ws"
    write_cache(ws, "gz-cmake", [
        "CMAKE_CXX_COMPILER:FILEPATH=/usr/bin/c++",
        f"CMAKE_C_COMPILER:FILEPATH={CONDA}/bin/arm64-apple-darwin20.0.0-clang",
    ])
    write_cache(ws, "gz-math", [
        f"CMAKE_CXX_COMPILER:STRING={CONDA}/bin/arm64-apple-darwin20.0.0-clang++",
        "CMAKE_C_COMPILER:STRING=/usr/bin/cc",
    ])
    assert leaks(ws)[0] == [
        "build/gz-cmake/CMakeCache.txt: CMAKE_CXX_COMPILER /usr/bin/c++",
        "build/gz-math/CMakeCache.txt: CMAKE_C_COMPILER /usr/bin/cc",
    ]
