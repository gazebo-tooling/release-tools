import shutil
from pathlib import Path

import pytest

from pixi_ci import packages
from pixi_ci.config import RELEASE_TOOLS, JobConfig
from pixi_ci.platforms import WindowsPlatform
from pixi_ci.runner import CIError

PLATFORM = WindowsPlatform()


@pytest.mark.parametrize("library, colcon_base, cmake_args, gpu", [
    ("gz-cmake", "gz-cmake", ("-DBUILDSYSTEM_TESTING:BOOL=True",), False),
    ("gz-fuel-tools", "gz-fuel_tools", (), False),
    ("gz-gui", "gz-gui", (), True),
    ("gz-rendering", "gz-rendering", (), True),
    ("gz-sensors", "gz-sensors", (), True),
    ("gz-sim", "gz-sim", (), True),
    ("gz-math", "gz-math", (), False),
    ("sdformat", "sdformat", (), False),
])
def test_real_collections_give_the_facts_of_each_library(library, colcon_base,
                                                         cmake_args, gpu):
    package = packages.get(library, RELEASE_TOOLS, PLATFORM)
    assert package.library == library
    assert package.colcon_base == colcon_base
    assert package.extra_cmake_args == cmake_args
    assert package.gpu is gpu


@pytest.mark.parametrize("metapackage", ["gz-harmonic", "gz-jetty", "gz-rotary"])
def test_real_collections_have_no_job_for_the_metapackages(metapackage):
    with pytest.raises(CIError, match="unknown library"):
        packages.get(metapackage, RELEASE_TOOLS, PLATFORM)


@pytest.fixture
def job_env(tmp_path):
    (tmp_path / "ws_root" / "gz-sim").mkdir(parents=True)
    (tmp_path / "rt" / "conda" / "envs" / "legacy").mkdir(parents=True)
    dsl = tmp_path / "rt" / "jenkins-scripts" / "dsl"
    dsl.mkdir(parents=True)
    shutil.copy(RELEASE_TOOLS / "jenkins-scripts" / "dsl" / "gz-collections.yaml",
                dsl)
    return {"WORKSPACE": str(tmp_path / "ws_root"),
            "PROGRAMDATA": str(tmp_path / "ProgramData")}


def config(env, tmp_path, library="gz-sim"):
    return JobConfig.from_environ(library, PLATFORM.normalize_env(env),
                                  PLATFORM, tmp_path / "rt")


def test_defaults(job_env, tmp_path):
    cfg = config(job_env, tmp_path)
    program_data = Path(job_env["PROGRAMDATA"])
    assert cfg.vcs_directory == "gz-sim"
    assert cfg.colcon_base == "gz-sim"
    assert cfg.auto_major is True
    assert cfg.build_type == "Release"
    assert cfg.enable_tests is True
    assert cfg.keep_workspace is False
    assert cfg.reuse_pixi is False
    assert cfg.gpu_needed is True
    assert cfg.conda_env_name is None
    assert cfg.gazebodistro_branch == "master"
    assert cfg.pixi_exe == program_data / "pixi" / "pixi.exe"
    assert cfg.project_path == program_data / "pixi" / "project"
    assert cfg.make_jobs >= 1


def test_variables_override_the_table(job_env, tmp_path):
    cfg = config(dict(job_env, GPU_SUPPORT_NEEDED="false", MAKE_JOBS="6",
                      COLCON_PACKAGE_EXTRA_CMAKE_ARGS='"-DA=1" -DB=2',
                      CONDA_ENV_NAME="legacy", BUILD_TYPE="Debug",
                      ghprbSourceBranch="ci_matching_branch/foo"), tmp_path)
    assert cfg.gpu_needed is False
    assert cfg.make_jobs == 6
    assert cfg.extra_cmake_args == ("-DA=1", "-DB=2")
    assert cfg.conda_env_name == "legacy"
    assert cfg.build_type == "Debug"
    assert cfg.pr_source_branch == "ci_matching_branch/foo"


def test_defined_means_set_and_non_empty_like_cmd(job_env, tmp_path):
    kept = config(dict(job_env, KEEP_WORKSPACE="false",
                       REUSE_PIXI_INSTALLATION="1"), tmp_path)
    empty = config(dict(job_env, KEEP_WORKSPACE="", CONDA_ENV_NAME=""), tmp_path)
    assert kept.keep_workspace is True      # defined, whatever the value
    assert kept.reuse_pixi is True
    assert empty.keep_workspace is False
    assert empty.conda_env_name is None


def test_enable_tests_accepts_any_case(job_env, tmp_path):
    assert config(dict(job_env, ENABLE_TESTS="true"), tmp_path).enable_tests
    assert not config(dict(job_env, ENABLE_TESTS="FALSE"), tmp_path).enable_tests


@pytest.mark.parametrize("variables, message", [
    ({"WORKSPACE": None}, "WORKSPACE is not set"),
    ({"VCS_DIRECTORY": "gz-math"}, "VCS_DIRECTORY points to"),
    ({"MAKE_JOBS": "auto"}, "MAKE_JOBS must be a positive integer"),
    ({"MAKE_JOBS": "0"}, "MAKE_JOBS must be a positive integer"),
    ({"CONDA_ENV_NAME": "noble_lik"}, "CONDA_ENV_NAME=noble_lik is not in"),
])
def test_bad_job_variables_fail_with_their_name(job_env, tmp_path, variables,
                                                message):
    env = {k: v for k, v in dict(job_env, **variables).items() if v is not None}
    with pytest.raises(CIError, match=message):
        config(env, tmp_path)


def test_describe_lists_values_and_the_variables_set(job_env, tmp_path):
    lines = config(dict(job_env, COLCON_PACKAGE="gz-math"), tmp_path).describe()
    assert "colcon_base: gz-math" in lines
    assert lines[-1].startswith("set in the job environment: ")
    assert "COLCON_PACKAGE=gz-math" in lines[-1]
