"""TEST ONLY: find what makes cl.exe report C4103 in gz-sim under Ninja.

Runs after the build of the package. For each object below it takes the
compile command from build.ninja and runs it again outside ninja:

* with /Zs under four INCLUDE variants, counting C4103: as activated, without
  the conda Library\\include that the vs2019 activation prepends, with the
  Windows SDK that the Visual Studio generator selected (10.0.19041.0), and
  both changes together;
* with /showIncludes as activated, printing the headers opened right before
  the first C4103;
* with /P, finding the files whose #pragma pack directives do not balance.

Nothing here can fail the job.
"""

import ntpath
import os
import re
import shutil
import subprocess
import traceback

OBJECTS = (
    "src/systems/physics/CMakeFiles/gz-sim-physics-system.dir/Physics.cc.obj",
    "src/systems/physics/CMakeFiles/UNIT_EntityFeatureMap_TEST.dir/"
    "EntityFeatureMap_TEST.cc.obj",
)
MSBUILD_SDK = "10.0.19041.0"
PACK = re.compile(r"(?:#\s*pragma\s+pack|__pragma\s*\(\s*pack)\s*\(([^)]*)\)")
LINE = re.compile(r'^#line\s+\d+\s+"(.*)"')


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


def include_variants(env):
    include = env.get("INCLUDE", "").split(";")
    conda_inc = ntpath.normcase(ntpath.join(env.get("CONDA_PREFIX", ""),
                                            "Library", "include"))
    without_conda = [p for p in include
                     if ntpath.normcase(p.rstrip("\\")) != conda_inc]
    sdk = env.get("WINDOWSSDKVERSION", "").strip("\\")

    def swap_sdk(paths):
        if not sdk or sdk == MSBUILD_SDK:
            return None
        swapped = [p.replace(f"\\{sdk}\\", f"\\{MSBUILD_SDK}\\") for p in paths]
        missing = [p for p in swapped if p and not os.path.isdir(p)]
        return None if missing else swapped

    variants = {"as activated": include, "without conda Library\\include":
                without_conda}
    for name, paths in (("SDK " + MSBUILD_SDK, include),
                        ("both", without_conda)):
        swapped = swap_sdk(paths)
        if swapped is not None:
            variants[name] = swapped
    return variants


def pack_report(preprocessed):
    """Files whose #pragma pack pushes and pops do not match, or that set
    the packing without a push."""
    stats, current = {}, "?"
    with open(preprocessed, encoding="utf-8", errors="replace") as f:
        for line in f:
            match = LINE.match(line)
            if match:
                current = match.group(1).replace("\\\\", "\\")
                continue
            for args in PACK.findall(line):
                words = [a.strip() for a in args.split(",") if a.strip()]
                entry = stats.setdefault(current, {"push": 0, "pop": 0,
                                                   "set": 0, "seen": []})
                kind = (words[0] if words and words[0] in ("push", "pop")
                        else "set")
                entry[kind] += 1
                if len(entry["seen"]) < 4:
                    entry["seen"].append(args.strip())
    return {name: s for name, s in stats.items()
            if s["push"] != s["pop"] or s["set"]}


def diagnose(build_dir, obj, env, ninja, out_dir):
    say(f"=== {obj}")
    command = compile_command(ninja, build_dir, obj, env)
    say("command:", command)
    base = re.sub(r'\s/Fo\S+', "", command)
    for name, paths in include_variants(env).items():
        venv = dict(env, INCLUDE=";".join(paths))
        code, out = run(base + " /Zs", build_dir, venv)
        warnings = [line for line in out.splitlines() if "C4103" in line]
        say(f"/Zs, INCLUDE {name}: exit {code}, {len(warnings)} C4103",
            *[f"  {w}" for w in warnings[:5]])
    code, out = run(base + " /Zs", build_dir, env)
    lines = out.splitlines()
    first = next((i for i, line in enumerate(lines) if "C4103" in line), None)
    if first is not None:
        say("headers opened before the first C4103 (/showIncludes):",
            *lines[max(0, first - 60):first + 1])
    stem = re.sub(r"[^A-Za-z0-9_.]", "_", obj)
    preprocessed = os.path.join(out_dir, stem + ".i")
    code, out = run(base + f' /P /Fi"{preprocessed}"', build_dir, env)
    if code != 0 or not os.path.isfile(preprocessed):
        say(f"/P failed ({code}): {out[-2000:]}")
        return
    unbalanced = pack_report(preprocessed)
    say(f"#pragma pack not balanced in {len(unbalanced)} files:")
    for name, s in unbalanced.items():
        say(f"  {name}: push {s['push']} pop {s['pop']} set {s['set']} "
            f"{s['seen']}")


def run_all(cfg, package, env):
    print("# BEGIN SECTION: TEST ONLY: C4103 diagnostics", flush=True)
    try:
        say("INCLUDE=" + env.get("INCLUDE", ""))
        for name in ("WINDOWSSDKVERSION", "UCRTVERSION", "VCTOOLSVERSION",
                     "WINDOWSSDKDIR", "CMAKE_GENERATOR"):
            say(f"{name}={env.get(name, '')}")
        sdk_dir = env.get("WINDOWSSDKDIR", "")
        if sdk_dir and os.path.isdir(os.path.join(sdk_dir, "Include")):
            say("SDKs: " + " ".join(sorted(os.listdir(
                os.path.join(sdk_dir, "Include")))))
        ninja = shutil.which("ninja", path=env.get("PATH"))
        build_dir = str(cfg.ws / "build" / package)
        out_dir = str(cfg.workspace / "c4103")
        os.makedirs(out_dir, exist_ok=True)
        for obj in OBJECTS:
            try:
                diagnose(build_dir, obj, env, ninja, out_dir)
            except Exception:
                say(traceback.format_exc())
    except Exception:
        say(traceback.format_exc())
    print("# END SECTION", flush=True)
