#!/usr/bin/env python3
"""
Unit tests for fuel_http.py, run against the in-process FakeFuel server.
"""

import http.client
import os
import socket
import ssl
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fuel_http
from fuel_http import FuelHttp, FuelHttpError, classify_error
from fake_fuel_server import FakeFuel

BIG = 3000000
MODELS = [
    ("openrobotics", "Mixed Case Model", BIG,
     {"/meshes/a.dae": BIG, "/model.sdf": 100, "/model.config": 50}),
    ("Some Owner", "other", 1000, {"/model.sdf": 10}),
]


def new_fuel(**kw):
    f = FakeFuel(**kw)
    f.models = list(MODELS)
    return f


def get(client, path, headers=None, **kw):
    return client.get(path, headers or {}, phase="t", klass="k",
                      t_sched=time.monotonic(), **kw)


class ClassifyErrorTest(unittest.TestCase):
    def test_classify_error_table(self):
        self.assertEqual(classify_error(429, None, None, 0), "throttled")
        self.assertEqual(classify_error(500, None, None, 0), "5xx")
        self.assertEqual(classify_error(503, None, None, 0), "5xx")
        self.assertEqual(classify_error(599, None, None, 0), "5xx")
        self.assertEqual(
            classify_error(None, socket.timeout("t"), None, 0), "timeout")
        self.assertEqual(
            classify_error(None, TimeoutError(), None, 0), "timeout")
        self.assertEqual(
            classify_error(None, ConnectionResetError(), None, 0), "conn")
        self.assertEqual(
            classify_error(None, http.client.RemoteDisconnected(), None, 0),
            "conn")
        self.assertEqual(classify_error(200, None, 100, 50), "short")
        self.assertEqual(classify_error(200, None, 100, 100), None)
        self.assertEqual(classify_error(200, None, None, 5), None)
        self.assertEqual(classify_error(404, None, None, 0), None)
        self.assertEqual(classify_error(403, None, None, 0), None)
        self.assertEqual(classify_error(200, None, None, 0), None)


class FuelHttpTest(unittest.TestCase):
    def setUp(self):
        self.fuel = new_fuel()
        self.fuel.__enter__()
        self.addCleanup(self.fuel.__exit__, None, None, None)
        self.client = FuelHttp(self.fuel.url, "unit", timeout_s=5)
        self.addCleanup(self.client.close)
        self.file = "/1.0/openrobotics/models/Mixed%20Case%20Model/tip/files/meshes/a.dae"

    def test_get_counts_bytes_and_marker(self):
        r = get(self.client, self.file)
        self.assertEqual(r.status, 200)
        self.assertEqual(r.nbytes, BIG)
        self.assertIsNone(r.error)
        self.assertEqual((r.phase, r.klass), ("t", "k"))
        path, headers = self.fuel.requests[-1]
        self.assertEqual(headers["X-Fuel-Load-Test"], "unit")

    def test_connection_is_reused(self):
        for _ in range(3):
            self.assertIsNone(get(self.client, "/1.0/models").error)
        self.assertEqual(len(self.fuel.requests), 3)
        self.assertEqual(self.fuel.connections, 1)

    def test_new_connection_after_error(self):
        get(self.client, "/1.0/models")
        self.fuel.status_override = 503
        get(self.client, "/1.0/models")  # 5xx keeps the connection
        self.assertEqual(self.fuel.connections, 1)
        self.fuel.status_override = None
        self.fuel.truncate = True
        self.assertEqual(get(self.client, self.file).error, "short")
        self.fuel.truncate = False
        self.assertIsNone(get(self.client, "/1.0/models").error)
        self.assertEqual(self.fuel.connections, 2)

    def test_truncated_chunked_body_is_short(self):
        self.fuel.chunked = True
        self.assertIsNone(get(self.client, self.file).error)
        self.fuel.truncate = True
        r = get(self.client, self.file)
        self.assertEqual(r.error, "short")
        self.assertEqual(r.status, 200)
        self.assertGreater(r.nbytes, 0)

    def test_get_never_raises(self):
        r = get(self.client, "/1.0/m\u00f6dels")
        self.assertEqual(r.error, "conn")
        self.assertIsNone(get(self.client, "/1.0/models").error)
        for url in ("http://host:abc/x", "http://[bad/x", "nonsense"):
            r = self.client.get_absolute(url, phase="t", klass="s3",
                                         t_sched=time.monotonic())
            self.assertEqual(r.error, "conn", url)

    def test_probe_size_does_not_read_body_without_range(self):
        class Resp:
            status = 200
            def getheader(self, name):
                return None
            def read(self, *a):
                raise AssertionError("body read")

        class Conn:
            def request(self, *a, **k):
                pass
            def getresponse(self):
                return Resp()
            def close(self):
                pass

        self.client._local.conn = Conn()
        self.assertIsNone(self.client.probe_size("/x"))
        self.assertIsNone(self.client._local.conn)

    def test_probe_size_takes_content_length_of_a_200(self):
        # R45: Range ignored; the size comes from Content-Length and the
        # body is never read.
        class Resp:
            status = 200
            def getheader(self, name):
                return {"Content-Length": "123456789012"}.get(name)
            def read(self, *a):
                raise AssertionError("body read")

        class Conn:
            sock = None
            def request(self, *a, **k):
                pass
            def getresponse(self):
                return Resp()
            def close(self):
                pass

        self.client._local.conn = Conn()
        self.assertEqual(self.client.probe_size("/x"), 123456789012)
        self.assertIsNone(self.client._local.conn)
        # and against the real fake, which ignores Range on zips
        zip_path = "/1.0/openrobotics/models/Mixed%20Case%20Model/tip/m.zip"
        self.assertEqual(self.client.probe_size(zip_path), BIG)
        self.assertIsNone(get(self.client, "/1.0/models").error)

    def test_connections_of_dead_threads_are_pruned(self):
        def worker():
            get(self.client, "/1.0/models")
        for _ in range(5):
            t = threading.Thread(target=worker)
            t.start()
            t.join()
        get(self.client, "/1.0/models")  # registers, prunes the dead
        self.assertLessEqual(len(self.client._conns), 2)
        self.client.close()
        self.assertEqual(len(self.client._conns), 0)

    def test_drop_conn_unregisters(self):
        self.fuel.down = True
        for _ in range(3):
            get(self.client, "/1.0/models")
        self.assertLessEqual(len(self.client._conns), 1)

    def test_concurrent_use(self):
        def one(_):
            return get(self.client, "/1.0/models").error
        with ThreadPoolExecutor(50) as ex:
            errors = list(ex.map(one, range(400)))
        self.assertEqual(set(errors), {None})
        self.assertLessEqual(self.fuel.connections, 50)

    def test_truncated_body_is_short(self):
        self.fuel.truncate = True
        r = get(self.client, self.file)
        self.assertEqual(r.error, "short")
        self.assertLess(r.nbytes, BIG)
        self.fuel.truncate = False
        self.assertIsNone(get(self.client, self.file).error)

    def test_down_server_is_conn_error(self):
        self.fuel.down = True
        r = get(self.client, "/1.0/models")
        self.assertEqual(r.error, "conn")
        self.assertIsNone(r.status)
        self.fuel.down = False
        self.assertIsNone(get(self.client, "/1.0/models").error)

    def test_timeout(self):
        self.fuel.delay_s = 1.0
        c = FuelHttp(self.fuel.url, "unit", timeout_s=0.2)
        self.addCleanup(c.close)
        self.assertEqual(get(c, "/1.0/models").error, "timeout")
        # The timed-out connection is dropped: the next request reconnects.
        self.fuel.delay_s = 0.0
        self.assertIsNone(get(c, "/1.0/models").error)
        self.assertEqual(self.fuel.connections, 2)

    def test_5xx_and_throttled(self):
        self.fuel.status_override = 503
        self.assertEqual(get(self.client, "/1.0/models").error, "5xx")
        self.fuel.status_override = 429
        self.assertEqual(get(self.client, "/1.0/models").error, "throttled")
        self.fuel.status_override = 404
        r = get(self.client, "/1.0/models")
        self.assertEqual((r.status, r.error), (404, None))

    def test_probe_size_reads_content_range(self):
        self.assertEqual(self.client.probe_size(self.file), BIG)
        self.assertEqual(self.fuel.requests[-1][1]["Range"], "bytes=0-0")
        self.assertIsNone(self.client.probe_size("/1.0/nope/models/x/tip/files/y"))

    def test_get_json_and_errors(self):
        data = self.client.get_json("/1.0/models?page=1&per_page=1")
        self.assertEqual(len(data), 1)
        self.assertEqual(data[0]["name"], "Mixed Case Model")
        with self.assertRaises(FuelHttpError):
            self.client.get_json("/1.0/models?page=99")

    def test_get_text(self):
        status, text = self.client.get_text(
            "/1.0/openrobotics/models/Mixed%20Case%20Model/tip/Mixed%20Case%20Model.zip?link=true", {})
        self.assertEqual(status, 200)
        self.assertEqual(text, "https://example-bucket.s3.amazonaws.com/x.zip")

    def test_capture_limit_keeps_body(self):
        url = "/1.0/openrobotics/models/Mixed%20Case%20Model/tip/Mixed%20Case%20Model.zip?link=true"
        r = get(self.client, url, capture_limit=4096)
        self.assertEqual(r.body, "https://example-bucket.s3.amazonaws.com/x.zip")
        self.assertIsNone(get(self.client, url).body)
        r = get(self.client, self.file, capture_limit=10)
        self.assertEqual(len(r.body), 10)
        self.assertEqual(r.nbytes, BIG)

    def test_get_absolute(self):
        r = self.client.get_absolute(self.fuel.url + self.file, phase="t",
                                     klass="s3", t_sched=time.monotonic())
        self.assertEqual((r.status, r.nbytes, r.error), (200, BIG, None))

    def test_conn_refused_is_conn_error(self):
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        c = FuelHttp("http://127.0.0.1:%d" % port, "unit", timeout_s=2)
        self.addCleanup(c.close)
        self.assertEqual(get(c, "/1.0/models").error, "conn")


class ProbeSizeWithoutRangeTest(unittest.TestCase):
    def test_probe_size_of_a_huge_file_without_range_support(self):
        huge = 10 ** 10  # 10 GB: reading the body would take many seconds
        fuel = FakeFuel(ignore_range=True)
        fuel.models = [("o", "m", huge, {"/meshes/huge.glb": huge})]
        with fuel:
            client = FuelHttp(fuel.url, "unit", timeout_s=5)
            try:
                t0 = time.monotonic()
                size = client.probe_size(
                    "/1.0/o/models/m/tip/files/meshes/huge.glb")
                elapsed = time.monotonic() - t0
                self.assertEqual(fuel.requests[-1][1]["Range"], "bytes=0-0")
                self.assertEqual(client.get_text("/1.0/models")[0], 200)
            finally:
                client.close()
        self.assertEqual(size, huge)
        self.assertLess(elapsed, 2.0)
        self.assertEqual(fuel.connections, 2)  # the probe's was dropped


class FakeFuelTest(unittest.TestCase):
    def setUp(self):
        self.fuel = new_fuel()
        self.fuel.__enter__()
        self.addCleanup(self.fuel.__exit__, None, None, None)
        self.client = FuelHttp(self.fuel.url, "unit", timeout_s=5)
        self.addCleanup(self.client.close)

    def test_fail_match_limits_failures(self):
        self.fuel.fail_rate = 1.0
        self.fuel.fail_match = r"/files/"
        r = get(self.client, "/1.0/openrobotics/models/Mixed%20Case%20Model/tip/files/model.sdf")
        self.assertEqual(r.status, 503)
        self.assertEqual(r.error, "5xx")
        r = get(self.client, "/1.0/models")
        self.assertEqual(r.status, 200)

    def test_shapes_and_separators(self):
        c = self.client
        tree = c.get_json("/1.0/openrobotics/models/Mixed%20Case%20Model/tip/files")
        top = {n["name"]: n for n in tree["file_tree"]}
        self.assertEqual(top["meshes"]["path"], "/meshes")
        self.assertEqual(top["meshes"]["children"][0]["path"], "/meshes/a.dae")
        self.assertFalse(top["model.sdf"].get("children"))
        for sep in ("/1.0/openrobotics%2Fmodels?page=1",
                    "/1.0/openrobotics%2fmodels?page=1",
                    "/1.0/openrobotics/models?page=1",
                    "/1.0/Some%20Owner/models"):
            self.assertTrue(c.get_json(sep), sep)
        info = c.get_json("/1.0/openrobotics%2Fmodels/Mixed%20Case%20Model")
        self.assertEqual(info["name"], "Mixed Case Model")
        for p in ("/1.0/collections", "/1.0/openrobotics/collections",
                  "/1.0/licenses"):
            self.assertEqual(get(c, p).status, 200, p)

    def test_head_is_405(self):
        conn = http.client.HTTPConnection(self.fuel.url[len("http://"):])
        conn.request("HEAD", "/1.0/models")
        r = conn.getresponse()
        r.read()
        self.assertEqual(r.status, 405)
        conn.close()


class StaleConnectionTest(unittest.TestCase):
    """R44: _simple_get retries once, on a fresh connection, when a kept-alive
    connection fails before any response byte."""

    def start(self, **kw):
        fuel = new_fuel(**kw)
        fuel.__enter__()
        self.addCleanup(fuel.__exit__, None, None, None)
        client = FuelHttp(fuel.url, "unit", timeout_s=5)
        self.addCleanup(client.close)
        return fuel, client

    def test_get_text_after_the_server_closed_an_idle_connection(self):
        fuel, client = self.start(idle_timeout_s=0.2)
        self.assertEqual(client.get_text("/1.0/models?per_page=1")[0], 200)
        time.sleep(0.6)  # the server closes the idle keep-alive connection
        status, text = client.get_text("/1.0/models?per_page=1")
        self.assertEqual(status, 200)
        self.assertIn("Mixed Case Model", text)
        self.assertEqual(fuel.connections, 2)
        self.assertEqual(client.get_json("/1.0/models?per_page=1")[0]["name"],
                         "Mixed Case Model")

    def test_reused_connection_is_retried_once(self):
        fuel, client = self.start()
        self.assertEqual(client.get_text("/1.0/models?per_page=1")[0], 200)
        fuel.down = True  # closes every connection that sends a request
        with self.assertRaises(FuelHttpError):
            client.get_text("/1.0/models?per_page=1")
        # The kept-alive attempt and one retry on a fresh connection.
        self.assertEqual(len(fuel.requests), 3)
        self.assertEqual(fuel.connections, 2)

    def test_ssl_eof_on_a_reused_connection_is_retried(self):
        # Over TLS, a reset received while the connection was idle makes
        # the send raise SSLEOFError instead of RemoteDisconnected.
        fuel, client = self.start()
        self.assertEqual(client.get_text("/1.0/models?per_page=1")[0], 200)

        def eof(*args, **kwargs):
            raise ssl.SSLEOFError(8, "EOF occurred in violation of protocol")

        client._conn().request = eof
        self.assertEqual(client.get_text("/1.0/models?per_page=1")[0], 200)
        self.assertEqual(fuel.connections, 2)

    def test_fresh_connection_is_not_retried(self):
        fuel, client = self.start(down=True)
        with self.assertRaises(FuelHttpError):
            client.get_text("/1.0/models?per_page=1")
        self.assertEqual(len(fuel.requests), 1)
        self.assertEqual(fuel.connections, 1)

    def test_load_path_get_is_not_retried(self):
        fuel, client = self.start(idle_timeout_s=0.2)
        self.assertIsNone(get(client, "/1.0/models").error)
        time.sleep(0.6)
        self.assertEqual(get(client, "/1.0/models").error, "conn")
        self.assertEqual(len(fuel.requests), 1)


class GetJsonErrorsTest(unittest.TestCase):
    def test_get_json_error_carries_status(self):
        with new_fuel() as fuel:
            client = FuelHttp(fuel.url, "t")
            try:
                with self.assertRaises(FuelHttpError) as cm:
                    client.get_json("/1.0/nope")
                self.assertEqual(cm.exception.status, 404)
            finally:
                client.close()

    def test_get_json_connection_error_has_no_status(self):
        with new_fuel(down=True) as fuel:
            client = FuelHttp(fuel.url, "t")
            try:
                with self.assertRaises(FuelHttpError) as cm:
                    client.get_json("/1.0/models")
                self.assertIsNone(cm.exception.status)
            finally:
                client.close()

    def test_get_json_conn_construction_failure_is_fuel_http_error(self):
        client = FuelHttp("http://127.0.0.1:1", "t")
        orig = fuel_http._make_conn
        fuel_http._make_conn = lambda *a: (_ for _ in ()).throw(
            OSError("boom"))
        try:
            with self.assertRaises(FuelHttpError):
                client.get_json("/1.0/models")
        finally:
            fuel_http._make_conn = orig
            client.close()


if __name__ == "__main__":
    unittest.main()
