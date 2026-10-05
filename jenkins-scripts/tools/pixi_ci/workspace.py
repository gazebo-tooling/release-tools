"""Colcon workspace: gazebodistro sources, sources under test, package name."""

from .fsops import copy_sources, remove_tree
from .runner import PYTHON, CIError, section

GAZEBODISTRO_URL = "https://github.com/gazebo-tooling/gazebodistro"


def setup(cfg):
    with section("setup workspace"):
        if not cfg.keep_workspace and cfg.ws.exists():
            with section("preclean workspace"):
                remove_tree(cfg.ws)
        (cfg.ws / "src").mkdir(parents=True, exist_ok=True)


def import_gazebodistro(runner, cfg, distro_file, env):
    distro_dir = cfg.workspace / "gazebodistro"
    remove_tree(distro_dir)
    runner.run("git", ["clone", GAZEBODISTRO_URL, distro_dir,
                       "-b", cfg.gazebodistro_branch],
               cwd=cfg.workspace, env=env)
    branch = cfg.pr_source_branch
    if branch is None:
        print("ghprbSourceBranch is unset", flush=True)
    else:
        script = (cfg.release_tools / "jenkins-scripts" / "tools" /
                  "detect_ci_matching_branch.py")
        matching = runner.run(PYTHON, [script, branch], cwd=cfg.workspace,
                              env=env, check=False)
        if matching.returncode == 0:
            print(f"trying to checkout branch {branch} from gazebodistro",
                  flush=True)
            # A missing branch in gazebodistro is fine: keep the default one
            runner.run("git", ["-C", distro_dir, "fetch", "origin", branch],
                       cwd=cfg.workspace, env=env, check=False)
            runner.run("git", ["-C", distro_dir, "checkout", branch],
                       cwd=cfg.workspace, env=env, check=False)
        else:
            print(f"branch name {branch} is not a match", flush=True)
        runner.run("git", ["-C", distro_dir, "branch"], cwd=cfg.workspace,
                   env=env, check=False)
    distro_path = distro_dir / distro_file
    if not distro_path.is_file():
        raise CIError(f"{distro_file} not found in gazebodistro "
                      f"(GAZEBODISTRO_BRANCH={cfg.gazebodistro_branch}); "
                      "set GAZEBODISTRO_FILE to the right file")
    runner.run("vcs", ["import", "--retry", "5", "--force",
                       "--input", distro_path, cfg.ws / "src"],
               cwd=cfg.workspace, env=env)
    runner.run("vcs", ["pull"], cwd=cfg.ws / "src", env=env)


def copy_sources_under_test(cfg, platform):
    destination = cfg.ws / "src" / cfg.vcs_directory
    with section(f"move {cfg.vcs_directory} source to workspace"):
        remove_tree(destination)
        copy_sources(cfg.sources, destination, platform.is_hidden_or_system)


def colcon_name_candidates(base, major, auto_major):
    """colcon names to look for, in order: Jetty and later drop the major
    version from package.xml, Fortress uses the ignition-* project() names."""
    candidates = [base + major] if auto_major else []
    candidates.append(base)
    if base == "gz-tools" and major == "1":
        candidates.append("ignition-tools")
    else:
        # Gazebo Fortress: gz-sim6 -> ignition-gazebo6
        candidates.append((base + major).replace("gz", "ignition")
                          .replace("sim", "gazebo"))
    return list(dict.fromkeys(candidates))


def resolve_colcon_name(available, base, major, auto_major):
    candidates = colcon_name_candidates(base, major, auto_major)
    for name in candidates:
        if name in available:
            return name
    raise CIError(f"Failed to find package {' or '.join(candidates)} in "
                  f"workspace: {', '.join(sorted(available))}")


def find_package(runner, cfg, major, env):
    """List the workspace and return the colcon name of the package under test."""
    with section("packages in workspace"):
        runner.run("colcon", ["list", "-t"], cwd=cfg.ws, env=env)
        runner.run("vcs", ["export", "--exact"], cwd=cfg.ws, env=env)
    first = cfg.colcon_base + major if cfg.auto_major else cfg.colcon_base
    with section(f"Check if package {first} is in colcon workspace"):
        listed = runner.run("colcon", ["list", "--names-only"], cwd=cfg.ws,
                            env=env, capture=True)
        print(f"Packages in workspace:\n{listed.stdout}", flush=True)
        name = resolve_colcon_name(set(listed.stdout.split()),
                                   cfg.colcon_base, major, cfg.auto_major)
        print(f"Using package name {name}", flush=True)
    return name
