"""A GNU make style jobserver shared by every Ninja the colcon builds start.

colcon builds several packages at the same time and each package runs its
own ninja, so a -j per ninja multiplies. Ninja 1.13 and newer is a jobserver
client: when MAKEFLAGS names a jobserver and ninja gets no -j, it takes a
token from the jobserver before starting each job beyond its first one. On
Windows the jobserver is a named semaphore holding the tokens.
"""

import contextlib
import os

from .runner import CIError


def makeflags(jobs, name):
    """MAKEFLAGS for colcon and ninja.

    The -j keeps colcon-cmake from passing -jN -lN to ninja: an explicit -j
    makes ninja ignore the jobserver.
    """
    return f"-j{jobs} --jobserver-auth={name}"


def tokens_for(slots):
    """Tokens that allow `slots` jobs to one ninja.

    Each ninja runs its first job without a token, like make does: one ninja
    alone runs up to `slots` jobs, and every other ninja building at the same
    time adds one job to that.
    """
    return max(0, slots - 1)


@contextlib.contextmanager
def windows_semaphore(tokens, name=None):
    """Yield the name of a semaphore holding `tokens`; close it on exit."""
    import ctypes
    from ctypes import wintypes

    name = name or f"pixi_ci_jobserver_{os.getpid()}"
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateSemaphoreW.restype = wintypes.HANDLE
    kernel32.CreateSemaphoreW.argtypes = (ctypes.c_void_p, wintypes.LONG,
                                          wintypes.LONG, wintypes.LPCWSTR)
    kernel32.CloseHandle.argtypes = (wintypes.HANDLE,)
    # The maximum count must be positive even without tokens
    handle = kernel32.CreateSemaphoreW(None, tokens, max(1, tokens), name)
    if not handle:
        raise CIError(f"CreateSemaphoreW({name}) failed: "
                      f"error {ctypes.get_last_error()}")
    try:
        yield name
    finally:
        kernel32.CloseHandle(handle)
