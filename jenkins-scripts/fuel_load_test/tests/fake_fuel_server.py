#!/usr/bin/env python3
"""
In-process fake of the Fuel server (staging shapes) for the load-test tests.
"""

import json
import random
import re
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BLOCK = b"x" * 65536


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 512

    def get_request(self):
        self.fuel.connections += 1
        return super().get_request()

    def handle_error(self, request, client_address):
        pass  # clients that give up (timeouts, resets) are expected


def _tree(files):
    root: dict = {}
    for path in sorted(files):
        node = root
        for part in path.strip("/").split("/"):
            node = node.setdefault(part, {})

    def build(children, prefix):
        out = []
        for name, sub in children.items():
            p = prefix + "/" + name
            item = {"name": name, "path": p}
            if sub:
                item["children"] = build(sub, p)
            out.append(item)
        return out

    return build(root, "")


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass

    def setup(self):
        # StreamRequestHandler sets self.timeout on the socket: with
        # idle_timeout_s, a keep-alive connection idle for longer is closed,
        # as a load balancer does.
        self.timeout = self.server.fuel.idle_timeout_s
        super().setup()

    # -- helpers -----------------------------------------------------------
    def _send(self, status, body=b"", ctype="application/json", extra=()):
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        for k, v in extra:
            self.send_header(k, v)
        self.end_headers()
        if body:
            self.wfile.write(body)

    def _send_json(self, obj, status=200):
        self._send(status, json.dumps(obj).encode())

    def _send_blob(self, size, start=0, end=None, status=200):
        """Send size-bytes of filler, honouring the truncate knob."""
        fuel = self.server.fuel
        length = (size if end is None else end - start + 1)
        if fuel.chunked:
            return self._send_chunked(length)
        extra = ()
        if end is not None:
            extra = (("Content-Range", "bytes %d-%d/%d" % (start, end, size)),)
        self.send_response(status)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(length))
        for k, v in extra:
            self.send_header(k, v)
        self.end_headers()
        to_send = length // 2 if fuel.truncate else length
        while to_send > 0:
            chunk = BLOCK[:min(to_send, len(BLOCK))]
            self.wfile.write(chunk)
            to_send -= len(chunk)
        if fuel.truncate:
            self.close_connection = True

    def _send_chunked(self, length):
        """Chunked body; with truncate, stop mid-stream without the final
        zero-length chunk."""
        fuel = self.server.fuel
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()
        to_send = length // 2 if fuel.truncate else length
        while to_send > 0:
            chunk = BLOCK[:min(to_send, len(BLOCK))]
            self.wfile.write(b"%x\r\n" % len(chunk) + chunk + b"\r\n")
            to_send -= len(chunk)
        if fuel.truncate:
            self.close_connection = True
        else:
            self.wfile.write(b"0\r\n\r\n")

    # -- routing -----------------------------------------------------------
    def do_HEAD(self):
        self.server.fuel.requests.append((self.path, dict(self.headers)))
        self._send(405)

    def do_GET(self):
        fuel = self.server.fuel
        with fuel._in_flight_lock:
            fuel.in_flight += 1
            fuel.max_in_flight = max(fuel.max_in_flight, fuel.in_flight)
        try:
            self._get()
        finally:
            with fuel._in_flight_lock:
                fuel.in_flight -= 1

    def _get(self):
        fuel = self.server.fuel
        fuel.requests.append((self.path, dict(self.headers)))
        if fuel.down:
            self.close_connection = True
            self.connection.close()
            return
        if fuel.delay_s:
            time.sleep(fuel.delay_s)
        if fuel.status_override is not None and (
                fuel.override_match is None
                or re.search(fuel.override_match, self.path)):
            self._send_json({"err": "override"}, fuel.status_override)
            return
        if fuel.fail_rate and (
                fuel.fail_match is None
                or re.search(fuel.fail_match, self.path)):
            if random.random() < fuel.fail_rate:
                self._send_json({"err": "injected"}, 503)
                return
        self._route()

    def _route(self):
        raw, _, query = self.path.partition("?")
        params = urllib.parse.parse_qs(query)
        if not raw.startswith("/1.0/"):
            return self._send_json({"err": "not found"}, 404)
        rest = re.sub(r"%2f", "/", raw[len("/1.0/"):], flags=re.I)
        seg = [urllib.parse.unquote(s) for s in rest.split("/")]
        fuel = self.server.fuel
        models = fuel.models

        if seg == ["models"]:
            return self._listing(models, params)
        if seg == ["collections"] or seg == ["licenses"]:
            return self._send_json([{"name": seg[0]}])
        if len(seg) == 2 and seg[1] == "collections":
            return self._send_json([{"name": "c", "owner": seg[0]}])
        if len(seg) == 2 and seg[1] == "models":
            owned = [m for m in models if m[0].lower() == seg[0].lower()]
            return self._listing(owned, params)
        if len(seg) >= 3 and seg[1] == "models":
            model = next((m for m in models if m[0].lower() == seg[0].lower()
                          and m[1].lower() == seg[2].lower()), None)
            if model is None:
                return self._send_json({"err": "no model"}, 404)
            return self._model(model, seg[3:], params)
        return self._send_json({"err": "not found"}, 404)

    def _listing(self, models, params):
        page = int(params.get("page", ["1"])[0])
        per_page = int(params.get("per_page", ["100"])[0])
        items = [_info(m) for m in models]
        chunk = items[(page - 1) * per_page:page * per_page]
        if page > 1 and not chunk:
            return self._send_json({"err": "past the end"}, 404)
        self._send_json(chunk)

    def _model(self, model, tail, params):
        owner, name, filesize, files = model
        if not tail:
            return self._send_json(_info(model))
        if tail[-1].endswith(".zip"):
            if params.get("link") == ["true"]:
                body = self.server.fuel.zip_link_body
                if body is None:
                    body = b"https://example-bucket.s3.amazonaws.com/x.zip"
                return self._send(200, body, "text/plain")
            return self._send_blob(filesize)
        if len(tail) >= 2 and tail[1] == "files":
            if len(tail) == 2:
                return self._send_json({"file_tree": _tree(files)})
            path = "/" + "/".join(tail[2:])
            norm = {("/" + k.lstrip("/")): v for k, v in files.items()}
            if path not in norm:
                return self._send_json({"err": "no file"}, 404)
            size = norm[path]
            rng = self.headers.get("Range")
            if self.server.fuel.ignore_range:
                rng = None  # a server that answers 200 with the whole file
            m = re.fullmatch(r"bytes=(\d+)-(\d*)", rng or "")
            if m:
                start = int(m.group(1))
                end = int(m.group(2)) if m.group(2) else size - 1
                return self._send_blob(size, start, min(end, size - 1), 206)
            return self._send_blob(size)
        return self._send_json({"err": "not found"}, 404)


def _info(model):
    owner, name, filesize, _ = model
    return {"owner": owner, "name": name, "filesize": filesize}


class FakeFuel:
    def __init__(self, fail_rate: float = 0.0, delay_s: float = 0.0,
                 status_override: int | None = None, truncate: bool = False,
                 down: bool = False, chunked: bool = False,
                 fail_match: str | None = None,
                 zip_link_body: bytes | None = None,
                 idle_timeout_s: float | None = None,
                 ignore_range: bool = False,
                 override_match: str | None = None):
        self.models: list = []
        self.fail_rate = fail_rate
        self.fail_match = fail_match
        self.delay_s = delay_s
        self.status_override = status_override
        self.override_match = override_match  # regex; None: every path
        self.truncate = truncate
        self.down = down
        self.chunked = chunked  # blobs use chunked encoding
        self.zip_link_body = zip_link_body  # None: an S3 link
        self.idle_timeout_s = idle_timeout_s  # None: keep-alive forever
        self.ignore_range = ignore_range  # /files/ answers Range with 200
        self.in_flight = 0  # GETs being handled now
        self.max_in_flight = 0  # high-water mark of in_flight
        self._in_flight_lock = threading.Lock()
        self.connections = 0  # accepted TCP connections
        self.requests: list = []
        self._server = _Server(("127.0.0.1", 0), _Handler)
        self._server.fuel = self
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True)

    @property
    def url(self) -> str:
        return "http://127.0.0.1:%d" % self._server.server_address[1]

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._server.shutdown()
        self._server.server_close()
