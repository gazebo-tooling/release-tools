"""TEST ONLY: find what makes cl.exe report C4103 in gz-sim under Ninja.

Round 2 (gz_sim-pr-cnlwin-nogpu #8 on citest) showed that the innermost
C4103 is at the first #include <atomic> (MSVC STL system_error:18), reached
after some gz-physics headers, whatever the command line. This round
compiles small probe files with the flags of each object and /Zs:

* trace: the first headers of the object, then <atomic>, with
  #pragma pack(show) (warning C4810 prints the packing value) after each
  include, and a report of the macros that could rewrite a #pragma pack
  (push, pop, pack, new, _CRT_PACKING);
* control: <atomic> alone;
* bisection: for each header that the first headers open (/showIncludes
  order), a probe with that header and then <atomic>; prints the headers
  after which <atomic> changes the packing.

Nothing here can fail the job.
"""

import ntpath
import os
import re
import shutil
import subprocess
import time
import traceback

# object -> the includes that come before the one that warns
OBJECTS = {
    "src/systems/physics/CMakeFiles/gz-sim-physics-system.dir/Physics.cc.obj":
        ("<memory>", "<unordered_map>", "<utility>",
         "<gz/physics/FindFeatures.hh>", "<gz/physics/RequestFeatures.hh>"),
    "src/systems/physics/CMakeFiles/UNIT_EntityFeatureMap_TEST.dir/"
    "EntityFeatureMap_TEST.cc.obj": ('"EntityFeatureMap.hh"',),
}
MAX_HEADERS = 400
BUDGET_SECONDS = 20 * 60
SHOW = re.compile(r"\((\d+)\).*C4810.*==\s*(\d+)")
NOTE = re.compile(r"Note: including file:\s*(.*\S)")
MACROS = ("push", "pop", "pack", "new", "show")


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


class Prober:
    """Compiles probe sources with the flags of one object."""

    def __init__(self, command, build_dir, env, out_dir, stem):
        flags, source = command.rsplit(" ", 1)
        flags = re.sub(r"\s/Fo\S+", "", flags).replace(" /showIncludes", "")
        self.flags = f'{flags} -I"{ntpath.dirname(source)}" /Zs'
        self.build_dir, self.env = build_dir, env
        self.out_dir, self.stem, self.count = out_dir, stem, 0

    def compile(self, lines, extra=""):
        self.count += 1
        path = os.path.join(self.out_dir, f"{self.stem}_{self.count}.cc")
        with open(path, "w", encoding="ascii", errors="replace") as f:
            f.write("\n".join(lines) + "\n")
        code, out = run(f'{self.flags} {extra} "{path}"', self.build_dir,
                        self.env)
        mine = os.path.basename(path).lower()
        shows = [(int(line_no), int(value))
                 for line_no, value in (m.groups() for m in
                                        (SHOW.search(line) for line in
                                         out.splitlines()) if m)]
        c4103 = [line for line in out.splitlines() if "warning C4103" in line]
        own = [line for line in c4103 if mine in line.lower()]
        return code, out, shows, c4103, own


def macro_lines():
    lines = ["#define C4103_STR2(x) #x", "#define C4103_STR(x) C4103_STR2(x)"]
    for name in MACROS:
        lines += [f"#ifdef {name}",
                  f'#pragma message("C4103 probe: {name} is a macro: "'
                  f" C4103_STR({name}))", "#endif"]
    lines.append('#pragma message("C4103 probe: _CRT_PACKING = "'
                 " C4103_STR(_CRT_PACKING))")
    return lines


def trace(prober, prefix):
    lines = ["#pragma pack(show)"]
    for include in prefix:
        lines += [f"#include {include}", "#pragma pack(show)"]
    lines += macro_lines()
    lines += ["#include <atomic>", "#pragma pack(show)"]
    code, out, shows, c4103, own = prober.compile(lines)
    say(f"trace: exit {code}, {len(c4103)} C4103 ({len(own)} in the probe)")
    for line_no, value in shows:
        say(f"  pack(show) after line {line_no - 1}: "
            f"{lines[line_no - 2] if line_no > 1 else '(start)'} -> {value}")
    say(*[f"  {line.strip()}" for line in out.splitlines()
          if "C4103 probe" in line or ("C4103" in line and "probe" in
                                       line.lower())][:20])
    code, out, shows, c4103, own = prober.compile(
        ["#pragma pack(show)", "#include <atomic>", "#pragma pack(show)"])
    say(f"control, <atomic> alone: exit {code}, {len(c4103)} C4103, "
        f"pack {[value for _, value in shows]}")


def is_system(path, env):
    roots = [env.get(name, "") for name in ("VCTOOLSINSTALLDIR",
                                             "WINDOWSSDKDIR")]
    return any(root and path.lower().startswith(root.lower())
               for root in roots)


def headers_of(prober, prefix):
    """(depth, path) of each file the prefix opens, first time only, the
    headers outside MSVC and the Windows SDK first."""
    lines = [f"#include {include}" for include in prefix]
    code, out, _, _, _ = prober.compile(lines, "/showIncludes")
    seen, ordered = set(), []
    for line in out.splitlines():
        match = NOTE.search(line)
        if match and match.group(1).lower() not in seen:
            seen.add(match.group(1).lower())
            rest = line.split("including file:", 1)[1]
            ordered.append((len(rest) - len(rest.lstrip()), match.group(1)))
    ordered.sort(key=lambda item: is_system(item[1], prober.env))
    return code, ordered


def bisect(prober, prefix):
    code, headers = headers_of(prober, prefix)
    say(f"bisection: the first headers open {len(headers)} files "
        f"(exit {code}); probing up to {MAX_HEADERS}, non-system first; "
        "hit = <atomic> after the header changes the packing")
    start, hits, failed = time.monotonic(), 0, 0
    for index, (depth, header) in enumerate(headers[:MAX_HEADERS]):
        if time.monotonic() - start > BUDGET_SECONDS:
            say(f"  time budget spent after {index} headers")
            break
        code, _, shows, _, own = prober.compile(
            ["#pragma pack(show)", f'#include "{header}"', "#pragma pack(show)",
             "#include <atomic>", "#pragma pack(show)"])
        failed += code != 0
        values = [value for _, value in shows]
        if own or len(set(values)) > 1:
            hits += 1
            say(f"  [{index}] depth {depth} {header}: exit {code}, "
                f"pack {values}, {len(own)} C4103 in the probe")
            if hits >= 150:
                say("  150 hits, stopping")
                break
    say(f"bisection done: {hits} hits, {failed} probes did not compile, "
        f"{time.monotonic() - start:.0f} s")


def diagnose(build_dir, obj, prefix, env, ninja, out_dir, stem):
    say(f"=== {obj}")
    command = compile_command(ninja, build_dir, obj, env)
    prober = Prober(command, build_dir, env, out_dir, stem)
    trace(prober, prefix)
    bisect(prober, prefix)


def run_all(cfg, package, env):
    print("# BEGIN SECTION: TEST ONLY: C4103 diagnostics", flush=True)
    try:
        ninja = shutil.which("ninja", path=env.get("PATH"))
        build_dir = str(cfg.ws / "build" / package)
        out_dir = str(cfg.workspace / "c4103")
        os.makedirs(out_dir, exist_ok=True)
        for number, (obj, prefix) in enumerate(OBJECTS.items()):
            try:
                diagnose(build_dir, obj, prefix, env, ninja, out_dir,
                         f"probe{number}")
            except Exception:
                say(traceback.format_exc())
    except Exception:
        say(traceback.format_exc())
    print("# END SECTION", flush=True)
