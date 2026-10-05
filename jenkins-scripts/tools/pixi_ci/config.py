"""Job configuration read from the Jenkins environment."""

import dataclasses
import os
import shlex
from dataclasses import dataclass
from pathlib import Path

from . import packages
from .runner import CIError

RELEASE_TOOLS = Path(__file__).resolve().parents[3]

# Job variables the driver reads (README.md describes each one)
VARIABLES = ("WORKSPACE", "VCS_DIRECTORY", "COLCON_PACKAGE",
             "COLCON_AUTO_MAJOR_VERSION", "COLCON_PACKAGE_EXTRA_CMAKE_ARGS",
             "BUILD_TYPE", "ENABLE_TESTS", "KEEP_WORKSPACE", "CONDA_ENV_NAME",
             "GAZEBODISTRO_FILE", "GAZEBODISTRO_BRANCH", "ghprbSourceBranch",
             "MAKE_JOBS", "GPU_SUPPORT_NEEDED", "REUSE_PIXI_INSTALLATION",
             "PIXI_TMP", "PIXI_PROJECT_PATH")


def _split_cmake_args(value):
    # cmd-style: "-DA=1" "-DB=2" -> -DA=1, -DB=2 (quotes removed)
    return tuple(token[1:-1] if len(token) > 1 and token[0] == token[-1] == '"'
                 else token
                 for token in shlex.split(value, posix=False))


def _positive_int(value, name):
    if not value.isdigit() or int(value) == 0:
        raise CIError(f"{name} must be a positive integer, got {value!r}")
    return int(value)


@dataclass(frozen=True)
class JobConfig:
    library: str
    release_tools: Path
    workspace: Path
    vcs_directory: str
    colcon_base: str
    auto_major: bool
    extra_cmake_args: tuple
    build_type: str
    enable_tests: bool
    keep_workspace: bool
    conda_env_name: str | None
    gazebodistro_file: str | None
    gazebodistro_branch: str
    pr_source_branch: str | None
    make_jobs: int
    gpu_needed: bool
    reuse_pixi: bool
    pixi_exe: Path
    project_path: Path
    job_variables: tuple = ()   # (name, value) of the VARIABLES that are set

    @property
    def sources(self):
        return self.workspace / self.vcs_directory

    @property
    def ws(self):
        return self.workspace / "ws"

    def describe(self):
        """Lines for the log: the effective values and the variables set."""
        lines = [f"{field.name}: {getattr(self, field.name)}"
                 for field in dataclasses.fields(self)
                 if field.name != "job_variables"]
        lines.append("set in the job environment: " + (", ".join(
            f"{name}={value}" for name, value in self.job_variables) or "none"))
        return lines

    @classmethod
    def from_environ(cls, library, env, platform, release_tools=RELEASE_TOOLS):
        """env is the job environment, already normalized by the platform."""
        def get(name):
            # cmd treats `set NAME=` as unset: so does an empty value here
            return env.get(platform.normalize_key(name)) or None

        package = packages.get(library)
        if get("WORKSPACE") is None:
            raise CIError("WORKSPACE is not set")
        workspace = Path(get("WORKSPACE"))
        vcs_directory = get("VCS_DIRECTORY") or library
        if not (workspace / vcs_directory).is_dir():
            raise CIError(f"VCS_DIRECTORY points to {workspace / vcs_directory}"
                          " but it does not exist")
        extra = get("COLCON_PACKAGE_EXTRA_CMAKE_ARGS")
        gpu = get("GPU_SUPPORT_NEEDED")
        jobs = get("MAKE_JOBS")
        conda_env_name = get("CONDA_ENV_NAME")
        if (conda_env_name is not None and
                not (release_tools / "conda" / "envs" / conda_env_name).is_dir()):
            raise CIError(f"CONDA_ENV_NAME={conda_env_name} is not in "
                          f"{release_tools / 'conda' / 'envs'}")
        return cls(
            library=library,
            release_tools=release_tools,
            workspace=workspace,
            vcs_directory=vcs_directory,
            colcon_base=get("COLCON_PACKAGE") or package.colcon_base,
            auto_major=(get("COLCON_AUTO_MAJOR_VERSION") or "true").lower() == "true",
            extra_cmake_args=(_split_cmake_args(extra) if extra is not None
                              else package.extra_cmake_args),
            build_type=get("BUILD_TYPE") or platform.default_build_type,
            enable_tests=(get("ENABLE_TESTS") or "TRUE").upper() == "TRUE",
            keep_workspace=get("KEEP_WORKSPACE") is not None,
            conda_env_name=conda_env_name,
            gazebodistro_file=get("GAZEBODISTRO_FILE"),
            gazebodistro_branch=get("GAZEBODISTRO_BRANCH") or "master",
            pr_source_branch=get("ghprbSourceBranch"),
            make_jobs=(_positive_int(jobs, "MAKE_JOBS") if jobs is not None
                       else os.cpu_count() or 1),
            gpu_needed=gpu.lower() == "true" if gpu is not None else package.gpu,
            reuse_pixi=get("REUSE_PIXI_INSTALLATION") is not None,
            pixi_exe=(Path(get("PIXI_TMP")) if get("PIXI_TMP")
                      else platform.default_pixi_exe(env)),
            project_path=(Path(get("PIXI_PROJECT_PATH")) if get("PIXI_PROJECT_PATH")
                          else platform.default_project_path(env)),
            job_variables=tuple((name, get(name)) for name in VARIABLES
                                if get(name) is not None),
        )
