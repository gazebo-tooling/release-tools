"""Per-library facts.

The libraries and their GPU needs come from gz-collections.yaml, the file that
generates the Jenkins jobs. The rest are the differences between the per-package
.bat files, which no file records.
"""

from dataclasses import dataclass

from get_ciconfigs_from_package_and_version import find_conda_libraries

from .runner import CIError

COLCON_BASES = {"gz-fuel-tools": "gz-fuel_tools"}
EXTRA_CMAKE_ARGS = {"gz-cmake": ("-DBUILDSYSTEM_TESTING:BOOL=True",)}


@dataclass(frozen=True)
class Package:
    colcon_base: str        # colcon name without major version
    extra_cmake_args: tuple
    gpu: bool               # needs an NVIDIA GPU (GPU_SUPPORT_NEEDED)


def libraries(release_tools, platform):
    """{library: needs a GPU} of the libraries with a job on this platform."""
    return find_conda_libraries(
        release_tools / "jenkins-scripts" / "dsl" / "gz-collections.yaml",
        platform.detect_os, platform.detect_arch)


def get(library, release_tools, platform):
    known = libraries(release_tools, platform)
    if library not in known:
        raise CIError(f"unknown library {library!r}, expected one of: "
                      f"{', '.join(sorted(known))}")
    return Package(COLCON_BASES.get(library, library),
                   EXTRA_CMAKE_ARGS.get(library, ()), known[library])
