"""Colcon workspace: gazebodistro sources, sources under test, package name."""

from .runner import CIError


def colcon_name_candidates(base, major, auto_major):
    """colcon names to look for, in order: Jetty and later drop the major
    version from package.xml, Fortress uses the ignition-* project() names."""
    candidates = [base + major] if auto_major else []
    candidates.append(base)
    if base == "gz-tools" and major == "1":
        candidates.append("ignition-tools")
    else:
        # Gazebo Fortress: gz-sim6 -> ignition-gazebo6
        candidates.append((base + major).replace("gz", "ignition")
                          .replace("sim", "gazebo"))
    return list(dict.fromkeys(candidates))


def resolve_colcon_name(available, base, major, auto_major):
    candidates = colcon_name_candidates(base, major, auto_major)
    for name in candidates:
        if name in available:
            return name
    raise CIError(f"Failed to find package {' or '.join(candidates)} in "
                  f"workspace: {', '.join(sorted(available))}")
