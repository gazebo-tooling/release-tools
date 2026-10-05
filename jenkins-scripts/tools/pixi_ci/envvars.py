"""Merge of the pixi activation variables and expansion of references."""

import re

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


def merge_activation(base, activated, platform):
    """base with the activated variables set over it, in order.

    Each value is expanded against the variables set so far, the way the
    lines of the hooks file are run one after another.
    """
    merged = dict(base)
    for key, value in platform.normalize_env(activated).items():
        merged[key] = platform.expand(value, merged)
    return merged
