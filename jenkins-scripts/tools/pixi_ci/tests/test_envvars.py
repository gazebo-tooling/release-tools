import pytest

from pixi_ci.envvars import expand_posix, expand_windows, merge_activation
from pixi_ci.platforms import WindowsPlatform
from pixi_ci.runner import CIError


def test_windows_expansion_is_case_insensitive():
    env = {"CONDA_PREFIX": r"C:\pixi\env", "PROGRAMFILES(X86)": r"C:\PF86"}
    assert expand_windows(r"%conda_prefix%\Library\plugins", env) == \
        r"C:\pixi\env\Library\plugins"
    assert expand_windows(r"%ProgramFiles(x86)%\Kits", env) == r"C:\PF86\Kits"


def test_windows_expansion_leaves_non_references_alone():
    assert expand_windows("50%20off 100%", {}) == "50%20off 100%"


def test_windows_undefined_reference_fails():
    with pytest.raises(CIError, match=r"undefined variable %NOPE%"):
        expand_windows(r"%NOPE%\bin", {})


def test_posix_expansion():
    env = {"CONDA_PREFIX": "/pixi/env"}
    assert expand_posix("$CONDA_PREFIX/lib:${CONDA_PREFIX}/plugins", env) == \
        "/pixi/env/lib:/pixi/env/plugins"
    with pytest.raises(CIError, match=r"undefined variable \$NOPE"):
        expand_posix("${NOPE}/bin", env)


def test_merge_expands_in_order_against_the_merged_env():
    platform = WindowsPlatform()
    base = platform.normalize_env({"Path": r"C:\Windows", "TEMP": r"C:\t"})
    activated = {
        "CONDA_PREFIX": r"C:\pixi\env",
        "Path": r"%CONDA_PREFIX%\Library\bin;%PATH%",
        "QT_QPA_PLATFORM_PLUGIN_PATH": r"%CONDA_PREFIX%\Library\plugins",
    }
    merged = merge_activation(base, activated, platform)
    assert merged == {
        "PATH": r"C:\pixi\env\Library\bin;C:\Windows",
        "TEMP": r"C:\t",
        "CONDA_PREFIX": r"C:\pixi\env",
        "QT_QPA_PLATFORM_PLUGIN_PATH": r"C:\pixi\env\Library\plugins",
    }


def test_post_activation_sets_the_ogre_paths():
    env = WindowsPlatform().post_activation({"CONDA_PREFIX": r"C:\pixi\env"})
    assert env["OGRE_RESOURCE_PATH"] == r"C:\pixi\env\Library\bin"
    assert env["OGRE2_RESOURCE_PATH"] == r"C:\pixi\env\Library\bin\OGRE-Next"
    with pytest.raises(CIError, match="CONDA_PREFIX is not set"):
        WindowsPlatform().post_activation({})
