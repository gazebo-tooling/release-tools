"""TEST ONLY: find what makes cl.exe report C4103 in gz-sim under Ninja.

Round 3 (gz_sim-pr-cnlwin-nogpu #9 on citest) found the trigger: after
gz-physics detail/RequestFeatures.hh, which has no #pragma, only the
out-of-class definitions of two member templates of RequestFeatures<>, the
next #pragma pack(show) reports two values, 16 and then 8 (_CRT_PACKING),
and the next #include <atomic> reports C4103. <atomic> alone does not.
This round compiles small probe files with the flags of Physics.cc.obj and
/Zs:

* base: the raw C4810 and C4103 lines of RequestFeatures.hh then <atomic>;
* layout: whether a struct declared after the header gets the packing 8;
* split: the two definitions of detail/RequestFeatures.hh one at a time;
* standalone: the same kind of definitions without the gz headers;
* flags: the base probe with one compiler flag changed at a time;
* generators: the base probe in a small CMake project built with Ninja and
  with Visual Studio 16 2019 (MSBuild), with the same compile options.

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
       "#include <utility>"]
A16 = ["#include <cstddef>", "struct alignas(16) C4103A { char c[16]; };"]

# gz-physics main, include/gz/physics/detail/RequestFeatures.hh
FROM = """
    /////////////////////////////////////////////////
    template <typename FeatureListT>
    template <
      typename PolicyT,
      typename FromFeatureList,
      template <typename, typename> class EntityT>
    EntityPtr<EntityT<PolicyT, FeatureListT>>
    RequestFeatures<FeatureListT>::From(
        const EntityPtr<EntityT<PolicyT, FromFeatureList>>& _from)
    {
      using ToPluginType =
          typename detail::DeterminePlugin<PolicyT, FeatureListT>::type;

      // If there is no underlying entity, then just return a null EntityPtr.
      if (!_from.entity)
        return nullptr;

      // If there is no underlying pimpl, then just return a null EntityPtr.
      // TODO(MXG): Should this ever be possible? Maybe this should be an
      // assertion instead?
      if (!_from.entity->pimpl)
        return nullptr;

      ToPluginType toPlugin(*_from.entity->pimpl);
      if (!detail::InspectFeatures<
              PolicyT,
              FeatureListT
          >::Verify(toPlugin))
      {
        // The physics plugin does not implement all of the features that were
        // requested, so we will return a nullptr.
        return nullptr;
      }

      return EntityPtr<EntityT<PolicyT, FeatureListT>>(
            std::make_shared<ToPluginType>(std::move(toPlugin)),
            _from.entity->identity);
    }
"""

MISSING = """
    /////////////////////////////////////////////////
    template <typename FeatureListT>
    template <
        typename PolicyT,
        typename FromFeatureList,
        template <typename, typename> class EntityT>
    std::set<std::string> RequestFeatures<FeatureListT>::MissingFeatureNames(
        const EntityPtr<EntityT<PolicyT, FromFeatureList>> &_entity)
    {
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

STANDALONE = {
    "out-of-class, class template, template template parameter": """
template <typename P, typename F> class C4103E {};
template <typename T> class C4103Ptr {};
template <typename L>
struct C4103R
{
  template <typename P, typename F, template <typename, typename> class E>
  static C4103Ptr<E<P, L>> From(const C4103Ptr<E<P, F>> &);
};
template <typename L>
template <typename P, typename F, template <typename, typename> class E>
C4103Ptr<E<P, L>> C4103R<L>::From(const C4103Ptr<E<P, F>> &)
{
  return {};
}
""",
    "out-of-class, class template, type parameters only": """
template <typename T> class C4103Ptr {};
template <typename L>
struct C4103R
{
  template <typename P, typename F>
  static C4103Ptr<P> From(const C4103Ptr<F> &);
};
template <typename L>
template <typename P, typename F>
C4103Ptr<P> C4103R<L>::From(const C4103Ptr<F> &)
{
  return {};
}
""",
    "in-class, class template, template template parameter": """
template <typename P, typename F> class C4103E {};
template <typename T> class C4103Ptr {};
template <typename L>
struct C4103R
{
  template <typename P, typename F, template <typename, typename> class E>
  static C4103Ptr<E<P, L>> From(const C4103Ptr<E<P, F>> &)
  {
    return {};
  }
};
""",
    "out-of-class, plain class, template template parameter": """
template <typename P, typename F> class C4103E {};
template <typename T> class C4103Ptr {};
struct C4103R
{
  template <typename P, typename F, template <typename, typename> class E>
  static C4103Ptr<E<P, F>> From(const C4103Ptr<E<P, F>> &);
};
template <typename P, typename F, template <typename, typename> class E>
C4103Ptr<E<P, F>> C4103R::From(const C4103Ptr<E<P, F>> &)
{
  return {};
}
""",
}

FLAG_VARIANTS = (
    ("without -permissive-", ("-permissive-",), ""),
    ("with /Zc:twoPhase-", (), "/Zc:twoPhase-"),
    ("without -Zc:__cplusplus", ("-Zc:__cplusplus",), ""),
    ("without -utf-8", ("-utf-8",), ""),
    ("without /GL", ("/GL",), ""),
    ("without /bigobj", ("/bigobj",), ""),
    ("-std:c++14", ("-std:c++17",), "-std:c++14"),
    ("-std:c++latest", ("-std:c++17",), "-std:c++latest"),
)

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

    def variant(self, stem, drop=(), add=""):
        other = Prober.__new__(Prober)
        other.__dict__.update(self.__dict__)
        other.stem, other.count = stem, 0
        flags = self.flags
        for token in drop:
            if token not in flags.split():
                say(f"    ({token} is not in the flags)")
            flags = re.sub(rf"(^|\s){re.escape(token)}(?=\s|$)", "", flags)
        other.flags = f"{flags} {add}".rstrip()
        return other

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
             if "C4103 probe" in line or ": error " in line
             or (raw and ("C4810" in line or "C4103" in line))]
    say(*[f"    {note}" for note in notes[:12]])


def wrap(code):
    return ["namespace gz", "{", "namespace physics", "{",
            *code.splitlines(), "}", "}"]


def layout(tag):
    return [f"struct C4103S_{tag} {{ char c; C4103A a; }};",
            (f"static_assert(offsetof(C4103S_{tag}, a) == 16, "
             f'"C4103 probe: {tag}: the alignas(16) member is not at 16");')]


def base(prober):
    say("base: RequestFeatures.hh then <atomic>, raw messages")
    report(prober, "base", BASE, raw=True)


def layouts(prober):
    say("layout: struct { char; alignas(16) member; } declared at each point; "
        "a static_assert error means the member offset is not 16")
    report(prober, "no header", A16 + layout("no_header"))
    report(prober, "inside pack(push, 8), positive control",
           A16 + ["#pragma pack(push, 8)"] + layout("pack8")
           + ["#pragma pack(pop)"])
    report(prober, "after RequestFeatures.hh and after <atomic>",
           A16 + ["#include <gz/physics/RequestFeatures.hh>",
                  "#pragma pack(show)"] + layout("after_header")
           + SHOW_ATOMIC + layout("after_atomic"))


def split(prober):
    say("split: RequestFeatures.hh without its detail header (include "
        "guard defined first), then the definitions of the detail header")
    head = ["#define GZ_PHYSICS_DETAIL_REQUESTFEATURES_HH_", *STL,
            "#include <gz/physics/RequestFeatures.hh>",
            "#include <gz/physics/detail/InspectFeatures.hh>",
            "#pragma pack(show)"]
    report(prober, "no definition", head + SHOW_ATOMIC)
    report(prober, "From", head + wrap(FROM) + SHOW_ATOMIC)
    report(prober, "MissingFeatureNames", head + wrap(MISSING) + SHOW_ATOMIC)
    report(prober, "both", head + wrap(FROM + MISSING) + SHOW_ATOMIC)


def standalone(prober):
    say("standalone: no gz headers; <memory> <set> <string> <utility>, "
        "the definition, then <atomic>")
    report(prober, "no definition", STL + SHOW_ATOMIC)
    for name, code in STANDALONE.items():
        report(prober, name,
               STL + ["#pragma pack(show)", *code.splitlines()] + SHOW_ATOMIC)
    first = next(iter(STANDALONE.values()))
    report(prober, "the first one without the STL headers before it",
           ["#pragma pack(show)", *first.splitlines()] + SHOW_ATOMIC)


def flag_variants(prober):
    say("flags: the base probe with one flag changed")
    for number, (name, drop, add) in enumerate(FLAG_VARIANTS):
        report(prober.variant(f"{prober.stem}f{number}", drop, add), name,
               BASE)


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
        # MSBuild prints file(line,column) and repeats the warnings at the end
        lines = list(dict.fromkeys(out.splitlines()))
        shows = [m.group(1) for m in map(VALUE.search, lines) if m]
        c4103 = [line for line in lines if "warning C4103" in line]
        say(f"  {name}: build exit {code}, pack {shows}, {len(c4103)} C4103")
        say(*[f"    {short(line)}" for line in c4103[:3]])
        say(*[f"    command: {line.strip()}" for line in lines
              if re.search(r"cl\.exe\s", line, re.IGNORECASE)][:2])
        if code != 0:
            say(out[-2000:])


def run_all(cfg, package, env):
    print("# BEGIN SECTION: TEST ONLY: C4103 diagnostics", flush=True)
    try:
        ninja = shutil.which("ninja", path=env.get("PATH"))
        build_dir = str(cfg.ws / "build" / package)
        out_dir = str(cfg.workspace / "c4103")
        os.makedirs(out_dir, exist_ok=True)
        command = compile_command(ninja, build_dir, OBJECT, env)
        prober = Prober(command, build_dir, env, out_dir, "probe")
        for step in (base, layouts, split, standalone, flag_variants):
            try:
                step(prober)
            except Exception:
                say(traceback.format_exc())
        generators(prober, command, env, ninja)
    except Exception:
        say(traceback.format_exc())
    print("# END SECTION", flush=True)
