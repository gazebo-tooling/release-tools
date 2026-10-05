"""Major version, env selection, env installation and activation."""

import json
import shutil

from . import envvars
from .fsops import remove_tree
from .runner import PYTHON, CIError, section


def detect_major_version(runner, cfg, env):
    script = (cfg.release_tools / "jenkins-scripts" / "tools" /
              "detect_cmake_major_version.py")
    cmakelists = cfg.sources / "CMakeLists.txt"
    result = runner.run(PYTHON, [script, cmakelists], cwd=cfg.workspace,
                        env=env, capture=True)
    major = result.stdout.strip()
    if not major.isdigit():
        raise CIError(f"no major version detected in {cmakelists}: {major!r}")
    print(f"MAJOR_VERSION detected: {major}", flush=True)
    return major


def select_env(runner, cfg, platform, major, env):
    """CONDA_ENV_NAME, or the env gz-collections.yaml gives this platform."""
    if cfg.conda_env_name is not None:
        print(f"Using user-specified conda environment: {cfg.conda_env_name}",
              flush=True)
        return cfg.conda_env_name
    dsl = cfg.release_tools / "jenkins-scripts" / "dsl"
    with section("auto-detect conda environment"):
        result = runner.run(
            PYTHON,
            [dsl / "tools" / "get_conda_ciconfig_from_package_and_version.py",
             "--yaml-file", dsl / "gz-collections.yaml",
             "--os", platform.detect_os, "--arch", platform.detect_arch,
             cfg.library, major],
            cwd=cfg.workspace, env=env, capture=True)
        name = result.stdout.strip()
        print(f"Detected conda environment: {name}", flush=True)
    if not name or not (cfg.release_tools / "conda" / "envs" / name).is_dir():
        raise CIError(f"detected conda environment {name!r} is not in "
                      f"{cfg.release_tools / 'conda' / 'envs'}")
    return name


def install_env(runner, cfg, env_name, env):
    """Recreate the pixi project from conda/envs/<env_name> and install it."""
    source = cfg.release_tools / "conda" / "envs" / env_name
    remove_tree(cfg.project_path)
    cfg.project_path.mkdir(parents=True)
    for manifest in sorted(source.glob("pixi.*")):
        shutil.copy2(manifest, cfg.project_path / manifest.name)
    runner.run(cfg.pixi_exe, ["install", "--locked"], cwd=cfg.project_path,
               env=env)


def check_reused_project(cfg):
    """REUSE_PIXI_INSTALLATION needs a project a previous run installed."""
    if not (cfg.project_path / "pixi.toml").is_file():
        raise CIError(f"REUSE_PIXI_INSTALLATION is set but {cfg.project_path} "
                      "has no pixi project: run once without it")
    if not cfg.pixi_exe.is_file():
        raise CIError(f"REUSE_PIXI_INSTALLATION is set but {cfg.pixi_exe} "
                      "does not exist: run once without it")


def show_env(runner, cfg, env):
    with section("pixi: info"):
        runner.run(cfg.pixi_exe, ["info"], cwd=cfg.project_path, env=env)
    with section("pixi: list packages"):
        runner.run(cfg.pixi_exe, ["list"], cwd=cfg.project_path, env=env)


def activate(runner, cfg, platform, env):
    """Write the hooks file for local use and return the activated env."""
    hooks = runner.run(cfg.pixi_exe,
                       ["shell-hook", "--locked", "--shell", platform.hook_shell],
                       cwd=cfg.project_path, env=env, capture=True)
    hooks_file = cfg.project_path / platform.hooks_file
    hooks_file.write_text(hooks.stdout, encoding="utf-8")
    print(hooks.stdout, flush=True)
    result = runner.run(cfg.pixi_exe, ["shell-hook", "--json", "--locked"],
                        cwd=cfg.project_path, env=env, capture=True)
    try:
        variables = json.loads(result.stdout)["environment_variables"]
    except (ValueError, KeyError, TypeError):
        raise CIError("unexpected output from pixi shell-hook --json: "
                      f"{result.stdout[:200]!r}") from None
    return envvars.merge_activation(env, variables, platform)
