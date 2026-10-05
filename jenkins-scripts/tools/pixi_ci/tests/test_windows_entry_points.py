"""The .bat files Jenkins runs on Windows end in the pixi_ci driver."""

import re
from pathlib import Path

from pixi_ci import packages

JENKINS_SCRIPTS = Path(__file__).resolve().parents[3]
LIB = JENKINS_SCRIPTS / "lib"
ENTRY_POINTS = sorted(JENKINS_SCRIPTS.glob("*-default-devel-windows-amd64.bat"))
REMOVED_LABELS = ("get_source_from_gazebodistro", "_colcon_build_cmd",
                  "build_workspace", "list_workspace_pkgs", "tests_in_workspace",
                  "pixi_create_gz_environment", "pixi_load_bootstrap_shell",
                  "pixi_load_shell", "pixi_cmd", "detect_conda_env")


def library_of(bat):
    return bat.name.replace("-default-devel-windows-amd64.bat", "").replace("_", "-")


def test_every_library_has_a_windows_entry_point():
    assert {library_of(bat) for bat in ENTRY_POINTS} == set(packages.PACKAGES)


def test_entry_points_pass_their_library_to_the_wrapper():
    for bat in ENTRY_POINTS:
        library = library_of(bat)
        text = bat.read_bytes().decode()
        assert (f"if not defined VCS_DIRECTORY set VCS_DIRECTORY={library}\r\n"
                in text), bat.name
        assert text.endswith(
            f'call "%SCRIPT_DIR%\\lib\\colcon-default-devel-windows.bat" '
            f'{library}\r\n'), bat.name


def test_wrapper_runs_the_driver_with_the_bootstrap_python():
    text = (LIB / "colcon-default-devel-windows.bat").read_bytes().decode()
    assert ('"%PIXI_BOOTSTRAP_PROJECT_PATH%\\.pixi\\envs\\default\\python.exe" '
            '-u "%SCRIPT_DIR%\\tools\\run_pixi_ci.py" %1 || goto :error'
            in text)
    assert "PYTHONPATH" not in text


def test_windows_library_keeps_only_the_labels_still_called():
    text = (LIB / "windows_library.bat").read_text()
    assert re.findall(r"^:([a-z_0-9]+)", text, re.MULTILINE) == [
        "configure_msvc2019_compiler", "wget", "retry", "pixi_installation",
        "pixi_create_bootstrap_environment", "pixi_bootstrap_cmd", "error"]


def test_no_script_uses_a_removed_label():
    for bat in sorted(JENKINS_SCRIPTS.rglob("*.bat")):
        text = bat.read_text(errors="replace")
        for label in REMOVED_LABELS:
            assert not re.search(rf":{label}\b", text), (bat.name, label)


def test_bat_files_keep_crlf_line_endings():
    for bat in sorted(JENKINS_SCRIPTS.rglob("*.bat")):
        data = bat.read_bytes()
        assert data.count(b"\n") == data.count(b"\r\n"), bat.name
