"""Merge of the pixi activation variables and expansion of references."""

import re
from collections.abc import Mapping

from .runner import CIError

_WINDOWS_REFERENCE = re.compile(r"%([A-Za-z_][A-Za-z0-9_()]*)%")
_POSIX_REFERENCE = re.compile(
    r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}|\$([A-Za-z_][A-Za-z0-9_]*)")


def expand_windows(value, env):
    """Expand %NAME% like cmd does. env keys are upper case."""
    def replace(match):
        name = match.group(1)
        if name.upper() not in env:
            raise CIError(f"undefined variable %{name}% in {value!r}")
        return env[name.upper()]
    return _WINDOWS_REFERENCE.sub(replace, value)


def expand_posix(value, env):
    """Expand $NAME and ${NAME} like sh does (macOS, Phase B)."""
    def replace(match):
        name = match.group(1) or match.group(2)
        if name not in env:
            raise CIError(f"undefined variable ${name} in {value!r}")
        return env[name]
    return _POSIX_REFERENCE.sub(replace, value)


class _Scope(Mapping):
    """The variables a reference in the value of `key` can name.

    Another activated variable gives its expanded value, wherever it is in
    the list: pixi prints the variables in a different order on every call.
    `key` itself (PATH=...;%PATH%) and the variables pixi does not set give
    their value in base.
    """

    def __init__(self, base, activated, key, resolve):
        self._base, self._activated = base, activated
        self._key, self._resolve = key, resolve

    def __getitem__(self, name):
        if name != self._key and name in self._activated:
            return self._resolve(name)
        return self._base[name]

    def __iter__(self):
        return iter(self._base.keys() | self._activated.keys())

    def __len__(self):
        return len(self._base.keys() | self._activated.keys())


def merge_activation(base, activated, platform):
    """base with the activated variables set over it, references expanded."""
    activated = platform.normalize_env(activated)
    expanded = {}
    in_progress = []

    def resolve(key):
        if key not in expanded:
            if key in in_progress:
                raise CIError("circular reference between the activation "
                              f"variables {' -> '.join(in_progress + [key])}")
            in_progress.append(key)
            expanded[key] = platform.expand(
                activated[key], _Scope(base, activated, key, resolve))
            in_progress.pop()
        return expanded[key]

    merged = dict(base)
    for key in activated:
        merged[key] = resolve(key)
    return merged
