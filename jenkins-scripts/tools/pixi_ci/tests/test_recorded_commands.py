"""The commands the driver runs for a Windows job, compared with the ones
jenkins-scripts/lib/colcon-default-devel-windows.bat runs today."""

from pathlib import Path

import yaml

from fakes import HOOKS, PREFIX, commands, find, make_job, option, sections
from pixi_ci.platforms import WindowsPlatform
from pixi_ci.runner import PYTHON, Result
from pixi_ci.workspace import GAZEBODISTRO_URL


def test_gz_math_jetty(tmp_path, capsys):
    job = make_job(tmp_path)
    code, calls = job.run()
    rt, ws_root, ws = job.rt, job.workspace, job.workspace / "ws"
    tools, dsl = rt / "jenkins-scripts" / "tools", rt / "jenkins-scripts" / "dsl"
    build = ["build", "--build-base", "build", "--install-base", "install",
             "--parallel-workers", "4"]
    handlers = ["--event-handler", "console_cohesion+", "desktop_notification-"]
    assert code == 0
    assert commands(calls) == [
        # .bat: python detect_cmake_major_version.py (COLCON_AUTO_MAJOR_VERSION)
        (PYTHON, [str(tools / "detect_cmake_major_version.py"),
                  str(ws_root / "gz-math" / "CMakeLists.txt")], ws_root),
        # :detect_conda_env, now with the library name and the platform
        (PYTHON, [str(dsl / "tools" / "get_ciconfigs_from_package_and_version.py"),
                  "--conda-env", "--os", "windows", "--arch", "amd64",
                  "--yaml-file", str(dsl / "gz-collections.yaml"),
                  "gz-math", "9"], ws_root),
        # :pixi_create_gz_environment (pixi install, now --locked)
        (job.pixi, ["install", "--locked"], job.project),
        (job.pixi, ["info"], job.project),
        (job.pixi, ["list"], job.project),
        # :pixi_load_shell writes hooks.bat; --json replaces `call hooks.bat`
        (job.pixi, ["shell-hook", "--locked", "--shell", "cmd"], job.project),
        (job.pixi, ["shell-hook", "--json", "--locked"], job.project),
        # :get_source_from_gazebodistro
        ("git", ["clone", GAZEBODISTRO_URL, str(ws_root / "gazebodistro"),
                 "-b", "master"], ws_root),
        ("vcs", ["import", "--retry", "5", "--force", "--input",
                 str(ws_root / "gazebodistro" / "gz-math9.yaml"),
                 str(ws / "src")], ws_root),
        ("vcs", ["pull"], ws / "src"),
        # :list_workspace_pkgs
        ("colcon", ["list", "-t"], ws),
        ("vcs", ["export", "--exact"], ws),
        # package name check
        ("colcon", ["list", "--names-only"], ws),
        # :build_workspace -> :_colcon_build_cmd, twice
        ("colcon", build + ["--packages-skip", "gz-math",
                            "--cmake-args", " -DCMAKE_BUILD_TYPE=Release",
                            "-DBUILD_TESTING=0", "-DCMAKE_CXX_FLAGS=-w"]
         + handlers, ws),
        ("colcon", build + ["--packages-select", "gz-math",
                            "--cmake-args", " -DCMAKE_BUILD_TYPE=Release",
                            " -DBUILD_TESTING=1"] + handlers, ws),
        # :tests_in_workspace
        ("colcon", ["test", "--install-base", "install",
                    "--packages-select", "gz-math", "--executor", "sequential",
                    "--event-handler", "console_direct+",
                    "desktop_notification-"], ws),
        ("colcon", ["test-result", "--all"], ws),
    ]
    assert sections(capsys) == [
        "pixi_ci configuration",
        "auto-detect conda environment",
        "pixi: create noble_like environment",
        "pixi: info",
        "pixi: list packages",
        "pixi: enable shell",
        "pixi: custom environment variable for gz",
        "setup workspace",
        "get open robotics deps (gz-math9.yaml) sources into the workspace",
        "move gz-math source to workspace",
        "packages in workspace",
        "Check if package gz-math9 is in colcon workspace",
        "compiling gz-math",
        "colcon compilation without test for dependencies of gz-math",
        "colcon compilation with tests for gz-math",
        "running tests for gz-math",
        "colcon test for gz-math",
        "colcon test-result",
        "export testing results",
        "clean up workspace",
    ]
    assert sorted(p.name for p in job.project.iterdir()) == [
        "hooks.bat", "pixi.lock", "pixi.toml"]
    assert (job.project / "hooks.bat").read_text() == HOOKS
    assert (ws_root / "build" / "test_results" / "gz-math" /
            "UNIT_Vector3_TEST.xml").is_file()
    assert not ws.exists()


def test_build_env_is_the_activated_one(tmp_path):
    job = make_job(tmp_path)
    code, calls = job.run()
    build_env = next(c.env for c in calls if c.args[0] == "build")
    assert code == 0
    assert build_env["CONDA_PREFIX"] == PREFIX
    assert build_env["PATH"] == PREFIX + r"\Library\bin;C:\Windows\system32"
    assert build_env["QT_QPA_PLATFORM_PLUGIN_PATH"] == PREFIX + r"\Library\plugins"
    assert build_env["OGRE_RESOURCE_PATH"] == PREFIX + r"\Library\bin"
    assert build_env["OGRE2_RESOURCE_PATH"] == PREFIX + r"\Library\bin\OGRE-Next"
    assert build_env["MAKEFLAGS"] == "-j4"
    assert all("PYTHONPATH" not in c.env for c in calls)
    # helper scripts and pixi run in the job environment, not the activated one
    assert all("CONDA_PREFIX" not in c.env for c in calls
               if c.tool in (PYTHON, job.pixi))


def test_sources_copy_leaves_hidden_entries_out(tmp_path, monkeypatch):
    monkeypatch.setattr(WindowsPlatform, "is_hidden_or_system",
                        lambda self, path: path.name.startswith("."))
    job = make_job(tmp_path)
    copied = []
    real_respond = job.respond

    def respond(call):
        if call.args[:2] == ["list", "-t"]:
            copied.extend(sorted(p.name for p in (call.cwd / "src" / "gz-math").iterdir()))
        return real_respond(call)
    job.respond = respond
    assert job.run()[0] == 0
    assert copied == ["CMakeLists.txt"]


def test_gz_cmake_passes_its_extra_cmake_arg(tmp_path):
    job = make_job(tmp_path, library="gz-cmake", major="5",
                   colcon_names=("gz-cmake",))
    code, calls = job.run()
    package_build = find(calls, "colcon", "build")[1].args
    cmake_args = package_build[package_build.index("--cmake-args"):]
    assert code == 0
    assert option(package_build, "--packages-select") == "gz-cmake"
    assert cmake_args[:4] == [
        "--cmake-args", " -DCMAKE_BUILD_TYPE=Release",
        "-DBUILDSYSTEM_TESTING:BOOL=True", " -DBUILD_TESTING=1"]


def test_gz_fuel_tools_maps_library_and_colcon_names(tmp_path):
    job = make_job(tmp_path, library="gz-fuel-tools", major="9",
                   colcon_names=("gz-cmake3", "gz-fuel_tools9"))
    code, calls = job.run()
    detection = [c for c in calls if c.tool == PYTHON][1].args
    vcs_import = find(calls, "vcs", "import")[0].args
    assert code == 0
    assert detection[-2:] == ["gz-fuel-tools", "9"]
    assert Path(option(vcs_import, "--input")).name == "gz-fuel-tools9.yaml"
    assert [option(c.args, "--packages-select")
            for c in find(calls, "colcon", "build")[1:]] == ["gz-fuel_tools9"]


def test_gz_sim_checks_the_gpu_first(tmp_path, capsys):
    job = make_job(tmp_path, library="gz-sim", major="10",
                   colcon_names=("gz-sim",))
    code, calls = job.run()
    assert code == 0
    assert commands(calls)[0] == (
        "dxdiag", ["/t", str(job.workspace / "dxdiag.txt")], job.workspace)
    assert sections(capsys)[1] == "dxdiag info"


def test_gz_gui_checks_the_gpu_as_its_job_label_requires(tmp_path):
    # gz-collections.yaml sends gz-gui to gpu-reliable agents; its .bat never
    # set GPU_SUPPORT_NEEDED
    job = make_job(tmp_path, library="gz-gui", major="10",
                   colcon_names=("gz-gui",))
    code, calls = job.run()
    assert code == 0
    assert calls[0].tool == "dxdiag"


def test_gz_math_needs_no_gpu(tmp_path):
    job = make_job(tmp_path, dxdiag="Manufacturer: Microsoft\n")
    code, calls = job.run()
    assert code == 0
    assert not any(c.tool == "dxdiag" for c in calls)


def test_gpu_support_needed_false_overrides_the_collections(tmp_path):
    job = make_job(tmp_path, library="gz-sim", major="10",
                   colcon_names=("gz-sim",), dxdiag="Manufacturer: Microsoft\n")
    code, calls = job.run(GPU_SUPPORT_NEEDED="false")
    assert code == 0
    assert not any(c.tool == "dxdiag" for c in calls)


def test_library_added_to_the_collections_needs_no_driver_change(tmp_path):
    job = make_job(tmp_path, library="gz-newlib",
                   colcon_names=("gz-cmake", "gz-newlib"))
    path = job.rt / "jenkins-scripts" / "dsl" / "gz-collections.yaml"
    data = yaml.safe_load(path.read_text())
    jetty = next(c for c in data["collections"] if c["name"] == "jetty")
    jetty["libs"].append({"name": "gz-newlib"})
    path.write_text(yaml.safe_dump(data))
    code, calls = job.run()
    assert code == 0
    assert [option(c.args, "--packages-select")
            for c in find(calls, "colcon", "build")[1:]] == ["gz-newlib"]


def test_unknown_library_fails_before_running_anything(tmp_path, capsys):
    job = make_job(tmp_path, library="gz-fuel_tools")
    code, calls = job.run()
    output = capsys.readouterr().out
    assert code == 1
    assert "ERROR: unknown library 'gz-fuel_tools'" in output
    assert "gz-fuel-tools" in output
    assert calls == []


def test_gz_sim_without_nvidia_gpu_fails_before_pixi(tmp_path, capsys):
    job = make_job(tmp_path, library="gz-sim", major="10",
                   colcon_names=("gz-sim",), dxdiag="Manufacturer: Microsoft\n")
    code, calls = job.run()
    assert code == 1
    assert [c.tool for c in calls] == ["dxdiag"]
    assert "ERROR: NVIDIA GPU not found in dxdiag" in capsys.readouterr().out


def test_fortress_library_uses_the_ignition_name(tmp_path):
    job = make_job(tmp_path, major="6",
                   colcon_names=("ignition-cmake2", "ignition-math6"))
    code, calls = job.run(CONDA_ENV_NAME="legacy")
    assert code == 0
    assert not any("get_ciconfigs" in a for c in calls for a in c.args)
    builds = [c.args for c in find(calls, "colcon", "build")]
    assert [option(builds[0], "--packages-skip"),
            option(builds[1], "--packages-select")] == [
        "ignition-math6", "ignition-math6"]
    assert (job.project / "pixi.toml").is_file()


def test_keep_workspace_keeps_it_before_and_after(tmp_path):
    job = make_job(tmp_path)
    (job.workspace / "ws" / "build").mkdir(parents=True)
    (job.workspace / "ws" / "build" / "previous").write_text("x")
    code, _ = job.run(KEEP_WORKSPACE="1")
    assert code == 0
    assert (job.workspace / "ws" / "build" / "previous").is_file()


def test_reuse_pixi_installation_skips_the_env_install(tmp_path):
    job = make_job(tmp_path)
    job.project.mkdir(parents=True)
    (job.project / "pixi.toml").write_text("[workspace]\n")
    (job.project / "previous").write_text("x")
    Path(job.pixi).write_text("")
    code, calls = job.run(REUSE_PIXI_INSTALLATION="1")
    pixi_args = [c.args for c in calls if c.tool == job.pixi]
    assert code == 0
    assert pixi_args == [["info"], ["list"],
                         ["shell-hook", "--locked", "--shell", "cmd"],
                         ["shell-hook", "--json", "--locked"]]
    assert (job.project / "previous").is_file()


def test_enable_tests_false_builds_with_tests_but_does_not_run_them(tmp_path,
                                                                    capsys):
    job = make_job(tmp_path)
    code, calls = job.run(ENABLE_TESTS="FALSE")
    assert code == 0
    assert [c.args[0] for c in calls if c.tool == "colcon"] == [
        "list", "list", "build", "build"]
    assert " -DBUILD_TESTING=1" in calls[-1].args
    assert "export testing results" not in sections(capsys)
    assert not (job.workspace / "build").exists()


def test_ci_matching_branch_checks_out_gazebodistro(tmp_path):
    job = make_job(tmp_path)
    distro = str(job.workspace / "gazebodistro")
    code, calls = job.run(ghprbSourceBranch="ci_matching_branch/my_feature")
    git = [c.args for c in calls if c.tool == "git"]
    assert code == 0
    assert git[1:] == [
        ["-C", distro, "fetch", "origin", "ci_matching_branch/my_feature"],
        ["-C", distro, "checkout", "ci_matching_branch/my_feature"],
        ["-C", distro, "branch"]]


def test_missing_gazebodistro_file_names_the_variable(tmp_path, capsys):
    job = make_job(tmp_path, distro_files=("gz-math8.yaml",))
    code, calls = job.run()
    assert code == 1
    assert ("ERROR: gz-math9.yaml not found in gazebodistro "
            "(GAZEBODISTRO_BRANCH=master); set GAZEBODISTRO_FILE") in \
        capsys.readouterr().out
    assert not any(c.tool == "vcs" for c in calls)


def test_failing_command_stops_the_job(tmp_path, capsys):
    job = make_job(tmp_path)
    job.replies.append((lambda c: c.args[:1] == ["import"], Result(1)))
    code, calls = job.run()
    assert code == 1
    assert calls[-1].args[0] == "import"
    assert "ERROR: exit code 1: vcs import --retry 5" in capsys.readouterr().out


# Failure modes a job meets that the cases above do not cover


def test_noise_around_the_shell_hook_json_fails_clearly(tmp_path, capsys):
    job = make_job(tmp_path)
    job.replies.append((lambda c: c.args[:2] == ["shell-hook", "--json"],
                        Result(0, "WARN the lock file is outdated\n{}")))
    code, calls = job.run()
    assert code == 1
    assert ("ERROR: unexpected output from pixi shell-hook --json: "
            "'WARN the lock file is outdated") in capsys.readouterr().out
    assert not any(c.tool in ("git", "vcs", "colcon") for c in calls)


def test_reuse_without_a_previous_installation_fails_clearly(tmp_path, capsys):
    job = make_job(tmp_path)
    code, calls = job.run(REUSE_PIXI_INSTALLATION="1")
    assert code == 1
    assert (f"ERROR: REUSE_PIXI_INSTALLATION is set but {job.project} has no "
            "pixi project") in capsys.readouterr().out
    assert not any(c.tool == job.pixi for c in calls)


def test_reuse_without_the_pixi_executable_fails_clearly(tmp_path, capsys):
    job = make_job(tmp_path)
    job.project.mkdir(parents=True)
    (job.project / "pixi.toml").write_text("[workspace]\n")
    code, calls = job.run(REUSE_PIXI_INSTALLATION="1")
    assert code == 1
    assert (f"ERROR: REUSE_PIXI_INSTALLATION is set but {job.pixi} does not "
            "exist") in capsys.readouterr().out
    assert not any(c.tool == job.pixi for c in calls)


def test_stale_job_variables_are_visible_in_the_log(tmp_path, capsys):
    job = make_job(tmp_path, library="gz-sim", major="10",
                   colcon_names=("gz-sim", "gz-math"))
    job.run(COLCON_PACKAGE="gz-math", CONDA_ENV_NAME="legacy")
    out = capsys.readouterr().out
    assert "colcon_base: gz-math" in out
    assert "COLCON_PACKAGE=gz-math" in out
    assert "CONDA_ENV_NAME=legacy" in out


def test_missing_test_results_fail_naming_the_directory(tmp_path, capsys):
    job = make_job(tmp_path)
    job.replies.append((lambda c: c.args[:1] == ["test"], Result(0)))
    code, _ = job.run()
    assert code == 1
    expected = job.workspace / "ws" / "build" / "gz-math" / "test_results"
    assert f"ERROR: no test results in {expected}" in capsys.readouterr().out


def test_ci_matching_branch_missing_in_gazebodistro_keeps_going(tmp_path):
    job = make_job(tmp_path)
    job.replies.append((lambda c: c.tool == "git" and "fetch" in c.args,
                        Result(128)))
    job.replies.append((lambda c: c.tool == "git" and "checkout" in c.args,
                        Result(1)))
    code, calls = job.run(ghprbSourceBranch="ci_matching_branch/not_there")
    assert code == 0
    assert find(calls, "vcs", "import")


def test_undetectable_major_version_fails_before_pixi(tmp_path, capsys):
    job = make_job(tmp_path)
    job.replies.append((lambda c: c.tool == PYTHON and
                        c.args[0].endswith("detect_cmake_major_version.py"),
                        Result(0, "")))
    code, calls = job.run()
    assert code == 1
    assert "ERROR: no major version detected in" in capsys.readouterr().out
    assert not any(c.tool == job.pixi for c in calls)
