"""TEST ONLY: find what makes cl.exe report C4103 in gz-sim under Ninja.

Round 4 (gz_sim-pr-cnlwin-nogpu #10 on citest) found:

* of the two definitions in gz-physics detail/RequestFeatures.hh, only
  RequestFeatures<>::MissingFeatureNames (returns std::set<std::string>)
  makes the next #pragma pack(show) report 16 and then 8, and the next
  <atomic> report C4103;
* it needs two-phase name lookup: no C4103 without -permissive-, with
  /Zc:twoPhase- or with -std:c++latest;
* an alignas(16) member stays at offset 16 inside #pragma pack(push, 8):
  on x64, packing 8 and 16 give the same layouts;
* a CMake project built by MSBuild with the same options does not warn.

This round compiles small probe files with the flags of Physics.cc.obj and
/Zs:

* missing: MissingFeatureNames written in other ways that a gz-physics
  change could use (trailing return type, an alias, an empty body);
* standalone: definitions returning std::set<std::string> without the gz
  headers, for a minimal reproduction;
* context: what before the header hides it (the round 4 layout probe had
  no C4103);
* generators: the raw messages of the Ninja and MSBuild builds, then the
  command that each one ran, run again outside the generator.

Nothing here can fail the job.
"""

import ntpath
import os
import re
import shutil
import subprocess
import traceback

OBJECT = "src/systems/physics/CMakeFiles/gz-sim-physics-system.dir/Physics.cc.obj"
SHOW = re.compile(r"\((\d+)\).*C4810.*==\s*(\d+)")
VALUE = re.compile(r"C4810.*==\s*(\d+)")
SHOW_ATOMIC = ["#pragma pack(show)", "#include <atomic>", "#pragma pack(show)"]
BASE = ["#pragma pack(show)", "#include <gz/physics/RequestFeatures.hh>",
        *SHOW_ATOMIC]
STL = ["#include <memory>", "#include <set>", "#include <string>",
       "#include <utility>", "#include <vector>"]

# gz-physics main, include/gz/physics/detail/RequestFeatures.hh
SIGNATURE = """
    /////////////////////////////////////////////////
    template <typename FeatureListT>
    template <
        typename PolicyT,
        typename FromFeatureList,
        template <typename, typename> class EntityT>
    std::set<std::string> RequestFeatures<FeatureListT>::MissingFeatureNames(
        const EntityPtr<EntityT<PolicyT, FromFeatureList>> &_entity)
"""
BODY = """    {
      std::set<std::string> names;
      if (!_entity.entity || !_entity.entity->pimpl)
      {
        detail::InspectFeatures<PolicyT, FeatureListT>::MissingNames(
              plugin::PluginPtr(), names);
        return names;
      }

      detail::InspectFeatures<PolicyT, FeatureListT>::MissingNames(
            *_entity.entity->pimpl, names);
      return names;
    }
"""
RETURN_TYPE = "    std::set<std::string> RequestFeatures"
MISSING = {
    "as is": SIGNATURE + BODY,
    "trailing return type": SIGNATURE.replace(
        RETURN_TYPE, "    auto RequestFeatures").replace(
        "&_entity)\n", "&_entity)\n    -> std::set<std::string>\n") + BODY,
    "return type through an alias":
        "    using C4103Names = std::set<std::string>;\n"
        + SIGNATURE.replace(RETURN_TYPE, "    C4103Names RequestFeatures")
        + BODY,
    "empty body": SIGNATURE + "    {\n      return {};\n    }\n",
}

SET_TT = """
template <typename P, typename F> class C4103E {};
template <typename T> class C4103Ptr {};
template <typename L>
struct C4103R
{
  template <typename P, typename F, template <typename, typename> class E>
  static std::set<std::string> Missing(const C4103Ptr<E<P, F>> &);
};
template <typename L>
template <typename P, typename F, template <typename, typename> class E>
std::set<std::string> C4103R<L>::Missing(const C4103Ptr<E<P, F>> &)
{
  std::set<std::string> names;
  return names;
}
"""
STANDALONE = {
    "class template, member template, template template parameter": SET_TT,
    "the same, return {}": SET_TT.replace(
        "  std::set<std::string> names;\n  return names;", "  return {};"),
    "the same, std::vector instead of std::set": SET_TT.replace(
        "std::set", "std::vector"),
    "class template, member template, type parameter": """
template <typename T> class C4103Ptr {};
template <typename L>
struct C4103R
{
  template <typename P>
  static std::set<std::string> Missing(const C4103Ptr<P> &);
};
template <typename L>
template <typename P>
std::set<std::string> C4103R<L>::Missing(const C4103Ptr<P> &)
{
  std::set<std::string> names;
  return names;
}
""",
    "class template, plain member function": """
template <typename L>
struct C4103R
{
  static std::set<std::string> Missing(int);
};
template <typename L>
std::set<std::string> C4103R<L>::Missing(int)
{
  std::set<std::string> names;
  return names;
}
""",
}

# compile options of the ninja command that the small CMake project sets
# itself or that do not apply to it
NOT_OPTIONS = ("/nologo", "/TP", "/showIncludes", "/FS", "-c")


def say(*lines):
    if lines:
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


def short(line):
    """A compiler message without the directory of the file."""
    return line.strip().rsplit("\\", 1)[-1][:300]


class Prober:
    """Compiles probe sources with the flags of one object."""

    def __init__(self, command, build_dir, env, out_dir, stem):
        flags, source = command.rsplit(" ", 1)
        flags = re.sub(r"\s/Fo\S+", "", flags).replace(" /showIncludes", "")
        self.source_dir = ntpath.dirname(source)
        self.flags = f'{flags} -I"{self.source_dir}" /Zs'
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


def report(prober, name, lines, raw=False):
    code, out, shows, c4103, own = prober.compile(lines)
    say(f"  {name}: exit {code}, pack {[value for _, value in shows]}, "
        f"{len(own)} C4103 in the probe, {len(c4103)} in all")
    notes = [short(line) for line in out.splitlines()
             if ": error " in line
             or (raw and ("C4810" in line or "C4103" in line))]
    say(*[f"    {note}" for note in notes[:12]])


def messages(out):
    """The C4810 values, the C4103 count and both raw, once each."""
    lines = list(dict.fromkeys(line.strip() for line in out.splitlines()
                               if "C4810" in line or "C4103" in line))
    values = [m.group(1) for m in map(VALUE.search, lines) if m]
    c4103 = [line for line in lines if "warning C4103" in line]
    return values, c4103, lines


def base(prober):
    say("base: RequestFeatures.hh then <atomic>, raw messages")
    report(prober, "base", BASE, raw=True)


def missing(prober):
    say("missing: RequestFeatures.hh without its detail header (include "
        "guard defined first), then MissingFeatureNames written as")
    head = ["#define GZ_PHYSICS_DETAIL_REQUESTFEATURES_HH_", *STL,
            "#include <gz/physics/RequestFeatures.hh>",
            "#include <gz/physics/detail/InspectFeatures.hh>",
            "#pragma pack(show)"]
    for name, code in MISSING.items():
        report(prober, name,
               head + ["namespace gz", "{", "namespace physics", "{",
                       *code.splitlines(), "}", "}"] + SHOW_ATOMIC)


def standalone(prober):
    say("standalone: no gz headers; the STL headers, the definition, then "
        "<atomic>")
    report(prober, "no definition", STL + SHOW_ATOMIC)
    for name, code in STANDALONE.items():
        report(prober, name,
               STL + ["#pragma pack(show)", *code.splitlines()] + SHOW_ATOMIC)


def context(prober):
    say("context: the base probe after other lines")
    report(prober, "without the first pack(show)", BASE[1:])
    report(prober, "after <cstddef>", ["#include <cstddef>", *BASE])
    report(prober, "after a plain struct",
           ["struct C4103B { char c; };", *BASE])
    report(prober, "after <cstddef> and an alignas(16) struct (round 4)",
           ["#include <cstddef>",
            "struct alignas(16) C4103A { char c[16]; };", *BASE])


def generators(prober, command, env, ninja):
    say("generators: a CMake OBJECT library with the base probe and the "
        "compile options of the ninja command")
    cmake = shutil.which("cmake", path=env.get("PATH"))
    if not cmake:
        say("  no cmake in PATH")
        return
    root = os.path.join(prober.out_dir, "generators")
    os.makedirs(root, exist_ok=True)
    with open(os.path.join(root, "probe.cc"), "w", encoding="ascii") as f:
        f.write("\n".join(BASE) + "\n")
    options = [token for token in command.split()[1:-1]
               if token not in NOT_OPTIONS
               and not token.startswith(("/Fo", "/Fd"))]
    options.append(f"-I{prober.source_dir}")
    with open(os.path.join(root, "CMakeLists.txt"), "w",
              encoding="ascii") as f:
        f.write("cmake_minimum_required(VERSION 3.16)\n"
                "project(c4103 CXX)\n"
                "add_library(c4103probe OBJECT probe.cc)\n"
                "target_compile_options(c4103probe PRIVATE\n"
                + "".join(f"  [==[{option}]==]\n" for option in options)
                + ")\n")
    clean = {key: value for key, value in env.items()
             if not key.upper().startswith("CMAKE_GENERATOR")}
    legs = (
        ("ninja", (f'-G Ninja -DCMAKE_BUILD_TYPE=Release '
                   f'-DCMAKE_MAKE_PROGRAM="{ninja}"'), "-- -v"),
        ("msbuild", '-G "Visual Studio 16 2019" -A x64 -T host=x64',
         "--config Release -- -v:n -nologo"),
    )
    for name, configure, build in legs:
        build_dir = os.path.join(root, f"build_{name}")
        code, out = run(f'"{cmake}" -S "{root}" -B "{build_dir}" {configure}',
                        root, clean)
        if code != 0:
            say(f"  {name}: configure exit {code}", out[-2000:])
            continue
        code, out = run(f'"{cmake}" --build "{build_dir}" {build}', root,
                        clean)
        values, c4103, lines = messages(out)
        say(f"  {name}: build exit {code}, pack {values}, {len(c4103)} C4103")
        say(*[f"    {short(line)}" for line in lines[:10]])
        if code != 0:
            say(out[-2000:])
            continue
        # the cl command that the generator ran, run again by itself; the
        # MSBuild log does not quote the path of CL.exe
        found = next(((line, m) for line, m in
                      ((line, re.search(r"cl\.exe\s+(?=/)", line,
                                        re.IGNORECASE))
                       for line in out.splitlines()) if m), None)
        if not found:
            say("    no cl command in the build output")
            continue
        line, match = found
        executable = re.sub(r"^\s*(\[\d+/\d+\])?\s*", "",
                            line[:match.start() + len("cl.exe")])
        code, out = run(f'"{executable}" {line[match.end():]}', build_dir,
                        env)
        values, c4103, lines = messages(out)
        say(f"  {name} command run alone: exit {code}, pack {values}, "
            f"{len(c4103)} C4103")
        say(*[f"    {short(line)}" for line in lines[:10]])


def run_all(cfg, package, env):
    print("# BEGIN SECTION: TEST ONLY: C4103 diagnostics", flush=True)
    try:
        ninja = shutil.which("ninja", path=env.get("PATH"))
        build_dir = str(cfg.ws / "build" / package)
        out_dir = str(cfg.workspace / "c4103")
        os.makedirs(out_dir, exist_ok=True)
        command = compile_command(ninja, build_dir, OBJECT, env)
        prober = Prober(command, build_dir, env, out_dir, "probe")
        for step in (base, missing, standalone, context):
            try:
                step(prober)
            except Exception:
                say(traceback.format_exc())
        generators(prober, command, env, ninja)
    except Exception:
        say(traceback.format_exc())
    print("# END SECTION", flush=True)
