"""Per-library facts: the differences between the per-package .bat files."""

from dataclasses import dataclass

from .runner import CIError


@dataclass(frozen=True)
class Package:
    library: str            # gz-collections.yaml name, e.g. gz-fuel-tools
    colcon_base: str        # colcon name without major version
    extra_cmake_args: tuple = ()
    gpu: bool = False       # needs an NVIDIA GPU (GPU_SUPPORT_NEEDED)


PACKAGES = {p.library: p for p in (
    Package("gz-cmake", "gz-cmake",
            extra_cmake_args=("-DBUILDSYSTEM_TESTING:BOOL=True",)),
    Package("gz-common", "gz-common"),
    Package("gz-fuel-tools", "gz-fuel_tools"),
    Package("gz-gui", "gz-gui"),
    Package("gz-launch", "gz-launch"),
    Package("gz-math", "gz-math"),
    Package("gz-msgs", "gz-msgs"),
    Package("gz-physics", "gz-physics"),
    Package("gz-plugin", "gz-plugin"),
    Package("gz-rendering", "gz-rendering", gpu=True),
    Package("gz-sensors", "gz-sensors", gpu=True),
    Package("gz-sim", "gz-sim", gpu=True),
    Package("gz-tools", "gz-tools"),
    Package("gz-transport", "gz-transport"),
    Package("gz-utils", "gz-utils"),
    Package("sdformat", "sdformat"),
)}


def get(library):
    try:
        return PACKAGES[library]
    except KeyError:
        raise CIError(f"unknown library {library!r}, expected one of: "
                      f"{', '.join(sorted(PACKAGES))}") from None
