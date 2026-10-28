#!/usr/bin/env python3
"""
HTTP client for the Fuel load test: one keep-alive connection per thread,
streamed bodies, and classification of failures.
"""

import http.client
import json
import re
import socket
import ssl
import threading
import time
import urllib.parse
from dataclasses import dataclass
from typing import Any

CHUNK = 1 << 20
# How a kept-alive connection that the server or the load balancer closed
# while it was idle fails, on the send or before any response byte.
# RemoteDisconnected is a ConnectionResetError; it is named for clarity.
# Over TLS, a reset received while idle fails the send with SSLEOFError.
_STALE_CONN_ERRORS = (http.client.RemoteDisconnected, ConnectionResetError,
                      ConnectionAbortedError, BrokenPipeError,
                      ssl.SSLEOFError)


class FuelHttpError(Exception):
    def __init__(self, message: str, status: int | None = None):
        super().__init__(message)
        self.status = status  # None for connection and parse errors


@dataclass
class RequestResult:
    t_sched: float
    t_start: float
    phase: str
    klass: str
    status: int | None
    nbytes: int
    latency_s: float
    error: str | None
    body: str | None = None


def classify_error(status: int | None, exc: BaseException | None,
                   content_length: int | None, nbytes: int) -> str | None:
    """Return None, "5xx", "throttled", "conn", "timeout" or "short"."""
    if exc is not None:
        if isinstance(exc, (socket.timeout, TimeoutError)):
            return "timeout"
        return "conn"
    if status == 429:
        return "throttled"
    if status is not None and 500 <= status <= 599:
        return "5xx"
    if content_length is not None and nbytes < content_length:
        return "short"
    return None


def _make_conn(scheme: str, netloc: str, timeout_s: float):
    cls = (http.client.HTTPSConnection if scheme == "https"
           else http.client.HTTPConnection)
    return cls(netloc, timeout=timeout_s)


class FuelHttp:
    def __init__(self, base_url: str, marker: str, timeout_s: float = 60.0):
        parsed = urllib.parse.urlsplit(base_url)
        self._scheme = parsed.scheme or "http"
        self._netloc = parsed.netloc
        self._prefix = parsed.path.rstrip("/")
        self._marker = marker
        self._timeout_s = timeout_s
        self._local = threading.local()
        self._conns: dict = {}  # conn -> owning thread
        self._conns_lock = threading.Lock()

    # -- connections -------------------------------------------------------
    def _conn(self):
        conn = getattr(self._local, "conn", None)
        if conn is None:
            conn = _make_conn(self._scheme, self._netloc, self._timeout_s)
            self._local.conn = conn
            with self._conns_lock:
                # Connections of exited threads (e.g. a previous phase's
                # executor) would otherwise stay open until close().
                dead = [c for c, t in self._conns.items()
                        if not t.is_alive()]
                for c in dead:
                    del self._conns[c]
                self._conns[conn] = threading.current_thread()
            for c in dead:
                c.close()
        return conn

    def _drop_conn(self):
        conn = getattr(self._local, "conn", None)
        self._local.conn = None
        if conn is not None:
            with self._conns_lock:
                self._conns.pop(conn, None)
            conn.close()

    def close(self):
        """Close every connection created by any thread."""
        with self._conns_lock:
            conns = list(self._conns)
            self._conns.clear()
        for conn in conns:
            conn.close()

    # -- core --------------------------------------------------------------
    def _headers(self, headers):
        h = dict(headers or {})
        h["X-Fuel-Load-Test"] = self._marker
        return h

    def _fetch(self, get_conn, path, headers, capture_limit):
        """Send a GET and stream the body. Returns (status, content_length,
        nbytes, captured_bytes, exc, truncated, conn). Never raises: any
        failure is returned as exc."""
        status = None
        content_length = None
        nbytes = 0
        captured = bytearray()
        exc = None
        truncated = False
        conn = None
        try:
            conn = get_conn()
            conn.request("GET", path, headers=self._headers(headers))
            resp = conn.getresponse()
            status = resp.status
            cl = resp.getheader("Content-Length")
            content_length = int(cl) if cl is not None and cl.isdigit() else None
            while True:
                try:
                    chunk = resp.read(CHUNK)
                except http.client.IncompleteRead as e:
                    # Raised for chunked bodies cut short.
                    chunk = e.partial
                    truncated = True
                if chunk:
                    nbytes += len(chunk)
                    if len(captured) < capture_limit:
                        captured += chunk[:capture_limit - len(captured)]
                if truncated or not chunk:
                    break
        except Exception as e:  # bad URL/header chars, sockets, HTTP errors
            exc = e
        return status, content_length, nbytes, bytes(captured), exc, \
            truncated, conn

    def _result(self, get_conn, path, headers, phase, klass, t_sched,
                capture_limit, pooled):
        t_start = time.monotonic()
        status, cl, nbytes, captured, exc, truncated, conn = self._fetch(
            get_conn, path, headers, capture_limit)
        latency = time.monotonic() - t_start
        error = classify_error(status, exc, cl, nbytes)
        if error is None and truncated:
            error = "short"
        if pooled:
            if error in ("conn", "timeout", "short"):
                self._drop_conn()
        elif conn is not None:
            conn.close()
        body = (captured.decode("utf-8", errors="replace")
                if capture_limit > 0 else None)
        return RequestResult(t_sched, t_start, phase, klass, status, nbytes,
                             latency, error, body)

    def get(self, path: str, headers: dict, *, phase: str, klass: str,
            t_sched: float, capture_limit: int = 0) -> RequestResult:
        return self._result(self._conn, self._prefix + path, headers, phase,
                            klass, t_sched, capture_limit, True)

    def get_absolute(self, url: str, *, phase: str, klass: str,
                     t_sched: float) -> RequestResult:
        def make():
            p = urllib.parse.urlsplit(url)
            return _make_conn(p.scheme, p.netloc, self._timeout_s)

        try:
            p = urllib.parse.urlsplit(url)
            path = (p.path or "/") + ("?" + p.query if p.query else "")
        except ValueError:
            path = "/"  # make() fails the same way and reports "conn"
        return self._result(make, path, {}, phase, klass, t_sched, 0, False)

    # -- helpers for discovery and health polls -----------------------------
    def _request(self, conn, path, headers):
        conn.request("GET", self._prefix + path,
                     headers=self._headers(headers))
        return conn.getresponse()

    def _simple_get(self, path, headers):
        """GET with the whole body read. When a kept-alive connection fails
        before any response byte (it was closed while idle), the request is
        retried once on a fresh connection; a fresh connection that fails is
        not retried. The load path, get(), never retries."""
        try:
            conn = self._conn()
            reused = conn.sock is not None  # still open from a previous GET
            try:
                resp = self._request(conn, path, headers)
            except _STALE_CONN_ERRORS:
                if not reused:
                    raise
                self._drop_conn()
                resp = self._request(self._conn(), path, headers)
            return resp.status, resp.read()
        except Exception as e:
            self._drop_conn()
            raise FuelHttpError("GET %s failed: %r" % (path, e)) from e

    def get_text(self, path: str, headers: dict | None = None
                 ) -> tuple[int, str]:
        status, body = self._simple_get(path, headers)
        return status, body.decode("utf-8", errors="replace")

    def get_json(self, path: str) -> Any:
        status, body = self._simple_get(path, {})
        if status != 200:
            raise FuelHttpError("GET %s returned %d" % (path, status),
                                status)
        try:
            return json.loads(body)
        except ValueError as e:
            raise FuelHttpError("GET %s: invalid JSON: %s" % (path, e)) from e

    def probe_size(self, path: str) -> int | None:
        """Total size of a resource using a one-byte range request: from
        Content-Range on a 206, or from Content-Length on a 200 (Range
        ignored). None otherwise."""
        try:
            conn = self._conn()
            conn.request("GET", self._prefix + path,
                         headers=self._headers({"Range": "bytes=0-0"}))
            resp = conn.getresponse()
            if resp.status != 206:
                # Range ignored (or an error): never read a possibly huge
                # body, just discard the connection.
                cl = resp.getheader("Content-Length")
                self._drop_conn()
                if resp.status == 200 and cl is not None and cl.isdigit():
                    return int(cl)
                return None
            cr = resp.getheader("Content-Range")
            resp.read()
            m = re.fullmatch(r"bytes \d+-\d+/(\d+)", (cr or "").strip())
            return int(m.group(1)) if m else None
        except Exception:
            self._drop_conn()
            return None
