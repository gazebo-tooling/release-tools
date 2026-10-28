#!/usr/bin/env python3
"""
Target guard: refuse to aim the load at anything but a staging Fuel server.
"""

import socket
from typing import Callable
from urllib.parse import urlsplit

PRODUCTION_HOST = "fuel.gazebosim.org"
ALLOWED_DOMAIN = ".gazebosim.org"  # a target host must end with it


class GuardError(Exception):
    pass


def resolve_ips(host: str) -> set[str]:
    return {info[4][0] for info in socket.getaddrinfo(host, None)}


def _resolve(resolve: Callable[[str], set[str]], host: str) -> set[str]:
    try:
        return set(resolve(host))
    except (OSError, UnicodeError) as e:  # gaierror is an OSError
        raise GuardError(
            f"rule resolution: cannot resolve {host!r} ({e}); refusing") from e


def check_target(url: str,
                 resolve: Callable[[str], set[str]] = resolve_ips) -> str:
    """Return the host if url is a safe staging target, else raise GuardError."""
    try:
        parts = urlsplit(url)
        host = (parts.hostname or "").lower().rstrip(".")
        parts.port  # raises ValueError on a bad port
    except ValueError as e:
        raise GuardError(f"rule url: invalid URL {url!r} ({e})") from e
    # The client connects with the raw netloc: refuse anything it could read
    # differently from the host validated here.
    if (parts.username is not None or "@" in parts.netloc
            or "\\" in parts.netloc
            or any(c.isspace() or ord(c) < 32 or ord(c) == 127
                   for c in url)):
        raise GuardError(
            f"rule netloc: userinfo, backslash, whitespace or control character in {url!r}")
    if parts.scheme != "https":
        raise GuardError(f"rule https: scheme must be https, got {url!r}")
    if not host:
        raise GuardError(f"rule host: no host in {url!r}")
    if "staging" not in host:
        raise GuardError(f"rule staging: host {host!r} does not contain 'staging'")
    if not host.endswith(ALLOWED_DOMAIN):
        raise GuardError(
            f"rule domain: host {host!r} is not under gazebosim.org")
    if host == PRODUCTION_HOST:
        raise GuardError(f"rule production: {host!r} is the production host")
    if _resolve(resolve, host) & _resolve(resolve, PRODUCTION_HOST):
        raise GuardError(
            f"rule production: {host!r} resolves to a production address")
    return host
