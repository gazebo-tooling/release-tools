"""lib/colcon-default-devel-macos.bash, with curl, shasum and pixi faked:
the pixi download and its check, the bootstrap env and the driver run.

The script runs with the bash of /usr/bin:/bin. On the macOS runners that is
bash 3.2, the /bin/bash of the agents, which ignores set -e for a failing
( ... ) subshell: the macOS pytest row is what covers it."""

import io
import os
import re
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(sys.platform == "win32",
                                reason="the macOS wrapper is a bash script")

JENKINS_SCRIPTS = Path(__file__).resolve().parents[3]
RELEASE_TOOLS = JENKINS_SCRIPTS.parent
WRAPPER = JENKINS_SCRIPTS / "lib" / "colcon-default-devel-macos.bash"
CONFIG_DETECTOR = RELEASE_TOOLS / "conda" / "config-detector"
PIXI_URL = ("https://github.com/prefix-dev/pixi/releases/download/v0.44.0/"
            "pixi-aarch64-apple-darwin.tar.gz")
# pixi-aarch64-apple-darwin.tar.gz.sha256 of the v0.44.0 release
PIXI_SHA256 = "56faaad85973c5739f46192ad8b62b64d6af391d4e3ded452f43dd5b603ea888"
SYSTEM_PATH = "/usr/bin:/bin"

FAKE_CURL = """#!/bin/bash
printf '%s\\n' "$@" >> "$FAKE_LOG/curl_args"
while [ $# -gt 0 ]; do
  if [ "$1" = -o ]; then cp "$FAKE_TARBALL" "$2"; fi
  shift
done
"""
FAKE_SHASUM = """#!/bin/bash
printf '%s\\n' "$*" >> "$FAKE_LOG/shasum_args"
cat >> "$FAKE_LOG/shasum_stdin"
exit "${FAKE_SHASUM_EXIT:-0}"
"""
# The pixi in the tarball; its install creates a python that records the
# driver run
FAKE_PIXI = """#!/bin/bash
printf '%s\\n' "$*" >> "$FAKE_LOG/pixi_calls"
case "$1" in
  --version) echo "pixi 0.44.0" ;;
  install)
    pwd -P >> "$FAKE_LOG/pixi_install_cwd"
    [ -z "${FAKE_PIXI_INSTALL_EXIT:-}" ] || exit "$FAKE_PIXI_INSTALL_EXIT"
    mkdir -p .pixi/envs/default/bin
    cat > .pixi/envs/default/bin/python <<'PYTHON'
#!/bin/bash
printf '%s\\n' "$@" > "$FAKE_LOG/driver_argv"
printf '%s\\n' "$PIXI_TMP" > "$FAKE_LOG/driver_pixi_tmp"
exit "${FAKE_DRIVER_EXIT:-0}"
PYTHON
    chmod +x .pixi/envs/default/bin/python ;;
esac
"""


def executable(path, text):
    path.write_text(text)
    path.chmod(0o755)


class Wrapper:
    def __init__(self, tmp_path):
        fakes = tmp_path / "fakes"
        fakes.mkdir()
        executable(fakes / "curl", FAKE_CURL)
        executable(fakes / "shasum", FAKE_SHASUM)
        tarball = tmp_path / "pixi-aarch64-apple-darwin.tar.gz"
        with tarfile.open(tarball, "w:gz") as tar:
            member = tarfile.TarInfo("pixi")
            member.size, member.mode = len(FAKE_PIXI.encode()), 0o755
            tar.addfile(member, io.BytesIO(FAKE_PIXI.encode()))
        self.log = tmp_path / "log"
        self.log.mkdir()
        self.workspace = tmp_path / "work space"
        self.workspace.mkdir()
        self.pixi = self.workspace / ".pixi_ci" / "pixi"
        self.bootstrap = self.workspace / ".pixi_ci" / "bootstrap"
        self.env = {"PATH": f"{fakes}:{SYSTEM_PATH}", "HOME": str(tmp_path),
                    "WORKSPACE": str(self.workspace),
                    "FAKE_LOG": str(self.log), "FAKE_TARBALL": str(tarball)}

    def run(self, *args, **variables):
        bash = shutil.which("bash", path=SYSTEM_PATH)
        return subprocess.run([bash, str(WRAPPER), *args],
                              env=dict(self.env, **variables),
                              capture_output=True, text=True)

    def logged(self, name):
        """Lines a fake wrote to the log, None if it never ran."""
        path = self.log / name
        return path.read_text().splitlines() if path.exists() else None


@pytest.fixture
def wrapper(tmp_path):
    return Wrapper(tmp_path)


def test_wrapper_installs_pixi_and_runs_the_driver(wrapper):
    result = wrapper.run("gz-math")
    assert result.returncode == 0, result.stdout + result.stderr
    assert wrapper.logged("curl_args")[-1] == PIXI_URL
    assert PIXI_SHA256 in "\n".join(wrapper.logged("shasum_stdin"))
    assert os.access(wrapper.pixi, os.X_OK)
    assert wrapper.pixi.read_text() == FAKE_PIXI
    assert wrapper.logged("pixi_calls") == ["--version", "install --locked"]
    assert wrapper.logged("pixi_install_cwd") == [str(wrapper.bootstrap.resolve())]
    for manifest in ("pixi.toml", "pixi.lock"):
        assert ((wrapper.bootstrap / manifest).read_bytes() ==
                (CONFIG_DETECTOR / manifest).read_bytes())
    assert wrapper.logged("driver_argv") == [
        "-u", "-I", str(JENKINS_SCRIPTS / "tools" / "run_pixi_ci.py"),
        "gz-math"]
    assert wrapper.logged("driver_pixi_tmp") == [str(wrapper.pixi)]
    assert "# BEGIN SECTION: pixi: installation" in result.stdout
    assert ("# BEGIN SECTION: pixi: create bootstrap environment"
            in result.stdout)


def test_partial_previous_download_is_replaced(wrapper):
    wrapper.bootstrap.mkdir(parents=True)
    wrapper.pixi.write_text("half a download")
    (wrapper.bootstrap / "pixi.toml").write_text("[workspace]\nstale = true\n")
    # what a run killed during the download leaves
    (wrapper.pixi.parent / "pixi.tar.gz.download").write_text("half")
    (wrapper.pixi.parent / "pixi.download").write_text("half")
    result = wrapper.run("gz-math")
    assert result.returncode == 0, result.stdout + result.stderr
    assert wrapper.pixi.read_text() == FAKE_PIXI
    assert ((wrapper.bootstrap / "pixi.toml").read_bytes() ==
            (CONFIG_DETECTOR / "pixi.toml").read_bytes())
    assert sorted(p.name for p in wrapper.pixi.parent.iterdir()) == [
        "bootstrap", "pixi"]


def test_checksum_mismatch_stops_before_replacing_pixi(wrapper):
    wrapper.pixi.parent.mkdir()
    wrapper.pixi.write_text("the previous pixi")
    result = wrapper.run("gz-math", FAKE_SHASUM_EXIT="1")
    assert result.returncode == 1
    assert ("ERROR: the pixi 0.44.0 download does not match PIXI_SHA256"
            in result.stdout)
    # the download ran and the check refused it
    assert wrapper.logged("curl_args")[-1] == PIXI_URL
    assert PIXI_SHA256 in "\n".join(wrapper.logged("shasum_stdin"))
    assert wrapper.pixi.read_text() == "the previous pixi"
    assert wrapper.logged("pixi_calls") is None
    assert wrapper.logged("driver_argv") is None


def test_failed_bootstrap_install_stops_the_job(wrapper):
    # the bootstrap env of an earlier run is still there
    assert wrapper.run("gz-math").returncode == 0
    (wrapper.log / "driver_argv").unlink()
    result = wrapper.run("gz-math", FAKE_PIXI_INSTALL_EXIT="1")
    assert result.returncode == 1
    assert wrapper.logged("pixi_calls")[-1] == "install --locked"
    assert (f"ERROR: pixi install --locked failed in {wrapper.bootstrap}"
            in result.stdout)
    assert wrapper.logged("driver_argv") is None


def test_reuse_skips_download_and_install(wrapper):
    assert wrapper.run("gz-math").returncode == 0
    for name in ("curl_args", "pixi_calls", "driver_argv"):
        (wrapper.log / name).unlink()
    result = wrapper.run("gz-math", REUSE_PIXI_INSTALLATION="1")
    assert result.returncode == 0, result.stdout + result.stderr
    assert wrapper.logged("curl_args") is None
    assert wrapper.logged("pixi_calls") is None
    assert wrapper.logged("driver_argv")[-1] == "gz-math"


def test_reuse_without_bootstrap_fails_clearly(wrapper):
    result = wrapper.run("gz-math", REUSE_PIXI_INSTALLATION="1")
    assert result.returncode == 1
    assert (f"ERROR: REUSE_PIXI_INSTALLATION is set but {wrapper.bootstrap} "
            "has no bootstrap environment: run once without it"
            in result.stdout)
    assert wrapper.logged("curl_args") is None


def test_driver_exit_code_is_the_job_result(wrapper):
    assert wrapper.run("gz-math", FAKE_DRIVER_EXIT="3").returncode == 3


def test_wrapper_needs_one_library_argument(wrapper):
    for args in ((), ("gz-math", "gz-sim")):
        result = wrapper.run(*args)
        assert result.returncode == 2
        assert "usage:" in result.stderr
    assert wrapper.logged("curl_args") is None


def test_wrapper_needs_the_workspace(wrapper):
    del wrapper.env["WORKSPACE"]
    result = wrapper.run("gz-math")
    assert result.returncode == 1
    assert "ERROR: WORKSPACE is not set" in result.stdout


def test_pixi_version_is_the_same_everywhere():
    found = {
        "windows_env_vars.bat": re.findall(
            r'set "PIXI_VERSION=([^"]+)"',
            (JENKINS_SCRIPTS / "lib" / "windows_env_vars.bat").read_text()),
        WRAPPER.name: re.findall(r"^PIXI_VERSION=(\S+)$", WRAPPER.read_text(),
                                 re.MULTILINE),
    }
    for workflow in sorted((RELEASE_TOOLS / ".github" / "workflows").glob("*.yaml")):
        versions = re.findall(r"pixi-version: v(\S+)", workflow.read_text())
        if versions:
            found[workflow.name] = versions
    assert len(found) >= 3, found
    assert all(len(versions) >= 1 for versions in found.values()), found
    assert len({v for versions in found.values() for v in versions}) == 1, found
