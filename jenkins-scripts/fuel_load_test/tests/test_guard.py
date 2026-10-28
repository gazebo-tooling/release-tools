#!/usr/bin/env python3
"""
Unit tests for guard.py. Name resolution is injected: no network.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import guard
from guard import GuardError, PRODUCTION_HOST, check_target

PROD_IP = "198.51.100.10"
STAGING_IP = "198.51.100.20"


def fake_resolve(host):
    return {PRODUCTION_HOST: {PROD_IP},
            "staging-fuel.gazebosim.org": {STAGING_IP}}[host]


class GuardTest(unittest.TestCase):
    def test_staging_allowed(self):
        self.assertEqual(
            check_target("https://staging-fuel.gazebosim.org", fake_resolve),
            "staging-fuel.gazebosim.org")

    def test_staging_with_port_and_case_allowed(self):
        self.assertEqual(
            check_target("https://Staging-Fuel.gazebosim.org.:8443/x",
                         fake_resolve),
            "staging-fuel.gazebosim.org")

    def test_production_refused(self):
        with self.assertRaises(GuardError):
            check_target("https://fuel.gazebosim.org", fake_resolve)

    def test_ip_literal_refused(self):
        with self.assertRaises(GuardError) as cm:
            check_target("https://198.51.100.30", fake_resolve)
        self.assertIn("staging", str(cm.exception))

    def test_staging_outside_gazebosim_org_refused(self):
        # R48: a third-party "staging" host is never a valid target, and is
        # refused before any name resolution.
        def resolve(host):
            raise AssertionError("resolved %r" % host)
        for url in ("https://staging.example.com",
                    "https://staging-fuel.gazebosim.org.example.com",
                    "https://staging-fuelgazebosim.org"):
            with self.subTest(url=url):
                with self.assertRaises(GuardError) as cm:
                    check_target(url, resolve)
                self.assertIn("gazebosim.org", str(cm.exception))

    def test_http_refused(self):
        with self.assertRaises(GuardError) as cm:
            check_target("http://staging-fuel.gazebosim.org", fake_resolve)
        self.assertIn("https", str(cm.exception))

    def test_no_host_refused(self):
        with self.assertRaises(GuardError):
            check_target("https://", fake_resolve)

    def test_userinfo_refused(self):
        for url in ("https://staging@fuel.gazebosim.org",
                    "https://staging-fuel.gazebosim.org@fuel.gazebosim.org",
                    "https://u:p@staging-fuel.gazebosim.org",
                    "https://staging-fuel.gazebosim.org\\@fuel.gazebosim.org",
                    "https://fuel.gazebosim.org\\staging-fuel.gazebosim.org"):
            with self.subTest(url=url), self.assertRaises(GuardError):
                check_target(url, fake_resolve)

    def test_whitespace_or_control_in_netloc_refused(self):
        for url in ("https://staging-fuel.gazebosim.org\t.x/",
                    "https://staging-fuel.gazebosim.org \n/",
                    "https://staging-fuel.gazebosim.org\x00/"):
            with self.subTest(url=url), self.assertRaises(GuardError):
                check_target(url, fake_resolve)

    def test_bad_port_refused(self):
        for url in ("https://staging-fuel.gazebosim.org:abc",
                    "https://staging-fuel.gazebosim.org:99999"):
            with self.subTest(url=url), self.assertRaises(GuardError):
                check_target(url, fake_resolve)

    def test_unicode_error_from_resolver_fails_closed(self):
        def resolve(host):
            raise UnicodeError("label too long")
        with self.assertRaises(GuardError):
            check_target("https://staging-fuel.gazebosim.org", resolve)

    def test_staging_cname_to_production_refused(self):
        with self.assertRaises(GuardError) as cm:
            check_target("https://staging-fuel.gazebosim.org",
                         lambda host: {PROD_IP})
        self.assertIn("production", str(cm.exception))

    def test_unresolvable_target_fails_closed(self):
        def resolve(host):
            if host == PRODUCTION_HOST:
                return {PROD_IP}
            raise OSError("no such host")
        with self.assertRaises(GuardError):
            check_target("https://staging-fuel.gazebosim.org", resolve)

    def test_unresolvable_production_fails_closed(self):
        def resolve(host):
            if host == PRODUCTION_HOST:
                raise OSError("no such host")
            return {STAGING_IP}
        with self.assertRaises(GuardError):
            check_target("https://staging-fuel.gazebosim.org", resolve)


if __name__ == "__main__":
    unittest.main()
