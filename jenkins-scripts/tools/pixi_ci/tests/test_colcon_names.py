import pytest

from pixi_ci import packages
from pixi_ci.runner import CIError
from pixi_ci.workspace import resolve_colcon_name

# Real colcon names of the supported branches. Fortress packages have no
# package.xml: colcon takes the CMake project() name (ignition-*)
FORTRESS = {"ignition-cmake2", "ignition-tools", "ignition-math6",
            "ignition-fuel_tools7", "ignition-gazebo6", "sdformat12"}


@pytest.mark.parametrize("base, major, expected", [
    ("gz-math", "6", "ignition-math6"),
    ("gz-sim", "6", "ignition-gazebo6"),
    ("gz-tools", "1", "ignition-tools"),
    ("gz-fuel_tools", "7", "ignition-fuel_tools7"),
    ("gz-cmake", "2", "ignition-cmake2"),
    ("sdformat", "12", "sdformat12"),
])
def test_fortress(base, major, expected):
    assert resolve_colcon_name(FORTRESS, base, major, True) == expected


@pytest.mark.parametrize("base, major, names, expected", [
    ("gz-sim", "8", {"gz-sim8", "gz-math7"}, "gz-sim8"),          # Harmonic
    ("gz-fuel_tools", "9", {"gz-fuel_tools9"}, "gz-fuel_tools9"),
    ("sdformat", "14", {"sdformat14"}, "sdformat14"),
    ("gz-tools", "2", {"gz-tools2", "gz-cmake"}, "gz-tools2"),    # Jetty
    ("gz-sim", "10", {"gz-sim", "gz-tools2"}, "gz-sim"),
    ("gz-sim", "11", {"gz-sim", "gz-tools"}, "gz-sim"),           # Rotary (M major)
])
def test_harmonic_jetty_rotary(base, major, names, expected):
    assert resolve_colcon_name(names, base, major, True) == expected


@pytest.mark.parametrize("package", sorted(packages.PACKAGES.values(),
                                           key=lambda p: p.library),
                         ids=lambda p: p.library)
def test_every_library_resolves_versioned_and_unversioned(package):
    base = package.colcon_base
    assert resolve_colcon_name({base}, base, "11", True) == base
    assert resolve_colcon_name({base + "9", base}, base, "9", True) == base + "9"


def test_names_match_exactly_not_as_substrings():
    # find "gz-sim1" in the .bat matched gz-sim10
    with pytest.raises(CIError, match="gz-sim1 or gz-sim or ignition-gazebo1"):
        resolve_colcon_name({"gz-sim10"}, "gz-sim", "1", True)


def test_without_auto_major_the_base_name_comes_first():
    assert resolve_colcon_name({"gz-math", "gz-math9"}, "gz-math", "9",
                               False) == "gz-math"
