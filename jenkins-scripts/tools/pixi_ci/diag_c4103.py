"""TEST ONLY: find what makes cl.exe report C4103 in gz-sim under Ninja.

Runs after the build of the package. For each object below it takes the
compile command from build.ninja and runs it again outside ninja with /Zs,
counting C4103, under variants of the command line that move it towards the
one the Visual Studio generator passes (which reports no C4103):

* without /showIncludes;
* with the -external:I directories as plain -I, and without -external:W0;
* with -external:W2, so that a C4103 inside an external header is shown;
* without the -I of the conda Library\\include that comes from CXXFLAGS (the
  same directory is also an -external:I);
* in the MSBuild order: the -I directories first, then -external:W0 and the
  -external:I directories after the other flags;
* with the flags that MSBuild adds (/GR /GS /fp:precise /Zc:inline ...);
* both of the last two.

The first round (INCLUDE without the conda Library\\include, /P scan of the
#pragma pack directives) is in the log of gz_sim-pr-cnlwin-nogpu #7 on citest.

Nothing here can fail the job.
"""

import ntpath
import re
import shutil
import subprocess
import traceback

OBJECTS = (
    "src/systems/physics/CMakeFiles/gz-sim-physics-system.dir/Physics.cc.obj",
    "src/systems/physics/CMakeFiles/UNIT_EntityFeatureMap_TEST.dir/"
    "EntityFeatureMap_TEST.cc.obj",
)
MSBUILD_FLAGS = ("/GR /Gm- /GS /fp:precise /Zc:wchar_t /Zc:forScope "
                 "/Zc:inline /Gd /FC /diagnostics:column")


def say(*lines):
    print("\n".join(str(line) for line in lines), flush=True)


def run(command, cwd, env):
    # A str goes to CreateProcess as is: no cmd.exe and its 8191 limit
    result = subprocess.run(command, cwd=cwd, env=env, capture_output=True,
                            text=True, errors="replace")
    return result.returncode, result.stdout + result.stderr


def compile_command(ninja, build_dir, obj, env):
    code, out = run(f'"{ninja}" -t commands -s {obj}', build_dir, env)
    if code != 0:
        code, out = run(f'"{ninja}" -t commands {obj}', build_dir, env)
    lines = [line for line in out.splitlines() if line.strip()]
    if code != 0 or not lines:
        raise RuntimeError(f"ninja -t commands failed ({code}): {out[-2000:]}")
    return lines[-1]


def is_external(token):
    return token.lower().startswith(("-external:i", "/external:i"))


def is_external_level(token):
    return token.lower().startswith(("-external:w", "/external:w"))


def is_include(token):
    return token.startswith(("-I", "/I"))


def is_conda_include(token, env):
    conda_inc = ntpath.normcase(ntpath.join(env.get("CONDA_PREFIX", ""),
                                            "Library", "include"))
    path = ntpath.normcase(token[2:].replace("/", "\\").rstrip("\\"))
    return is_include(token) and path == conda_inc


def msbuild_order(tokens):
    """cl.exe first, then -I, the other flags, -external:W0, -external:I."""
    cl, rest = tokens[0], tokens[1:]
    includes = [t for t in rest if is_include(t)]
    level = [t for t in rest if is_external_level(t)]
    external = [t for t in rest if is_external(t)]
    others = [t for t in rest if not (is_include(t) or is_external_level(t)
                                      or is_external(t))]
    return [cl, *includes, *others, *level, *external]


def variants(command, env):
    base = re.sub(r'\s/Fo\S+', "", command)
    result = {
        "as built": base,
        "without /showIncludes": base.replace(" /showIncludes", ""),
        "-external:I as -I, no -external:W0": re.sub(
            r"(?<=\s)[-/]external:I", "-I",
            re.sub(r"\s[-/]external:W0", "", base)),
        "-external:W2": re.sub(r"(?<=\s)([-/]external:)W0", r"\1W2", base),
    }
    if '"' in base:
        say("quoted arguments in the command: no token variants")
        return result
    tokens = base.split()
    result["without the CXXFLAGS -I conda Library\\include"] = " ".join(
        t for t in tokens if not is_conda_include(t, env))
    result["MSBuild order"] = " ".join(msbuild_order(tokens))
    result["MSBuild flags"] = base + " " + MSBUILD_FLAGS
    result["MSBuild order and flags"] = (" ".join(msbuild_order(tokens)) +
                                         " " + MSBUILD_FLAGS)
    return result


def diagnose(build_dir, obj, env, ninja):
    say(f"=== {obj}")
    command = compile_command(ninja, build_dir, obj, env)
    say("command:", command)
    for name, variant in variants(command, env).items():
        code, out = run(variant + " /Zs", build_dir, env)
        warnings = [line for line in out.splitlines() if "C4103" in line]
        say(f"/Zs, {name}: exit {code}, {len(warnings)} C4103",
            *[f"  {w}" for w in warnings[:10]])
        if code != 0:
            say(*[f"  | {line}" for line in out.splitlines()
                  if "Note: including file" not in line][:10])


def run_all(cfg, package, env):
    print("# BEGIN SECTION: TEST ONLY: C4103 diagnostics", flush=True)
    try:
        ninja = shutil.which("ninja", path=env.get("PATH"))
        build_dir = str(cfg.ws / "build" / package)
        for obj in OBJECTS:
            try:
                diagnose(build_dir, obj, env, ninja)
            except Exception:
                say(traceback.format_exc())
    except Exception:
        say(traceback.format_exc())
    print("# END SECTION", flush=True)
