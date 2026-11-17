"""Merge of the pixi activation variables and expansion of references."""

import graphlib
import re

from .runner import CIError

_WINDOWS_REFERENCE = re.compile(r"%([A-Za-z_][A-Za-z0-9_()]*)%")


def windows_references(value):
    """Upper case names of the %NAME% references in value."""
    return {name.upper() for name in _WINDOWS_REFERENCE.findall(value)}


def expand_windows(value, env):
    """Expand %NAME% like cmd does. env keys are upper case."""
    def replace(match):
        name = match.group(1)
        if name.upper() not in env:
            raise CIError(f"undefined variable %{name}% in {value!r}")
        return env[name.upper()]
    return _WINDOWS_REFERENCE.sub(replace, value)


def merge_activation(base, activated, platform):
    """base with the activated variables set over it, references expanded.

    pixi prints the variables in a different order on every call, so they are
    expanded in dependency order: a reference to another activated variable
    gives its expanded value. A variable that references itself
    (PATH=...;%PATH%) and the variables pixi does not set give their value in
    base.
    """
    activated = platform.normalize_env(activated)
    dependencies = {key: (platform.references(value) & activated.keys()) - {key}
                    for key, value in activated.items()}
    try:
        order = list(graphlib.TopologicalSorter(dependencies).static_order())
    except graphlib.CycleError as error:
        raise CIError("circular reference between the activation variables "
                      f"{' -> '.join(error.args[1])}") from None
    merged = dict(base)
    for key in order:
        merged[key] = platform.expand(activated[key], merged)
    return merged
