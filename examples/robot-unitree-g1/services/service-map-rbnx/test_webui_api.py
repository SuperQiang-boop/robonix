# SPDX-License-Identifier: MulanPSL-2.0
"""The web UI's HTTP surface, exercised against stub impls.

The page is not a second implementation of the map operations -- it calls the
same map_ops functions the gRPC servicers and the MCP handlers call. The bug
this guards against is the page diverging from them: carrying its own copy of
the session state, or passing arguments the other entry points do not. Each
test asserts the handler forwards to map_ops and reports back what map_ops (or
lifecycle) says, rather than anything the page remembered.
"""
from __future__ import annotations

import json
import re
import threading
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from unittest.mock import patch

from mapping_rbnx import webui


class WebUiApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), webui._Handler)
        cls.base = "http://127.0.0.1:%d" % cls.srv.server_address[1]
        cls.thread = threading.Thread(target=cls.srv.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def call(self, path, body=None):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(
            self.base + path, data=data,
            headers={"Content-Type": "application/json"},
            method="POST" if data is not None else "GET")
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read() or b"{}")

    def test_save_passes_no_session_state_of_its_own(self):
        # A page that supplied its own live-database hint here is exactly how
        # the two entry points drifted apart. It must call the impl the same
        # way the servicers do: id and note, nothing else.
        with patch.object(webui.map_ops, "save_map_impl",
                          return_value={"ok": True, "detail": "saved"}) as save:
            out = self.call("/api/save", {"map_id": "lab", "note": "n"})
        self.assertTrue(out["ok"])
        save.assert_called_once_with("lab", "n")

    def test_load_forwards_the_pose_seed(self):
        with patch.object(webui.map_ops, "load_map_impl",
                          return_value={"ok": True, "detail": "loaded"}) as load:
            self.call("/api/load", {"map_id": "lab", "mode": "localization",
                                    "has_initial_pose": True,
                                    "x": 1.5, "y": -2.0, "theta": 0.25})
        load.assert_called_once_with("lab", "localization", True, 1.5, -2.0, 0.25)

    def test_state_reports_the_service_mode_not_a_local_copy(self):
        with (
            patch.object(webui.map_ops, "get_mode_impl",
                         return_value={"ok": True, "mode": "localization", "detail": ""}),
            patch.object(webui.lifecycle, "current",
                         return_value={"map_id": "lab_3f", "mode": "localization"}),
        ):
            out = self.call("/api/state")
        self.assertEqual(out["mode"], "localization")
        self.assertEqual(out["map_id"], "lab_3f")

    def test_state_mode_follows_a_change_the_page_never_made(self):
        # A load or a reset issued over MCP changes the mode with the page
        # uninvolved; the badge has to follow it.
        for mode in ("mapping", "localization"):
            with (
                patch.object(webui.map_ops, "get_mode_impl",
                             return_value={"ok": True, "mode": mode, "detail": ""}),
                patch.object(webui.lifecycle, "current",
                             return_value={"map_id": "", "mode": mode}),
            ):
                self.assertEqual(self.call("/api/state")["mode"], mode)

    def test_switch_and_reset_reach_the_impls(self):
        with patch.object(webui.map_ops, "switch_mode_impl",
                          return_value={"ok": True, "detail": "switched"}) as sw:
            self.call("/api/switch_mode", {"mode": "mapping"})
        sw.assert_called_once_with("mapping")
        with patch.object(webui.map_ops, "reset_map_impl",
                          return_value={"ok": True, "detail": "cleared"}) as rs:
            self.call("/api/reset", {})
        rs.assert_called_once_with()

    def test_a_failing_impl_is_reported_verbatim(self):
        # The page must not soften or invent a result: the operator needs the
        # service's own words to act on.
        with patch.object(webui.map_ops, "save_map_impl",
                          return_value={"ok": False, "detail": "no live rtabmap database"}):
            out = self.call("/api/save", {"map_id": "lab"})
        self.assertFalse(out["ok"])
        self.assertEqual(out["detail"], "no live rtabmap database")

    def test_every_action_is_recorded_in_the_activity_log(self):
        with (
            patch.object(webui.map_ops, "save_map_impl", return_value={"ok": True, "detail": "saved lab"}),
            patch.object(webui.map_ops, "reset_map_impl", return_value={"ok": True, "detail": "cleared"}),
        ):
            self.call("/api/save", {"map_id": "lab"})
            self.call("/api/reset", {})
        kinds = {e["kind"] for e in self.call("/api/log")}
        self.assertTrue({"save", "reset"} <= kinds, kinds)


class StaticAssetTest(unittest.TestCase):
    """The page is three files on disk, not a Python string.

    It used to be one string holding minified HTML, CSS and JavaScript. That
    made every change a merge hazard and silently ate escapes: a "\\n" written
    for a JS string literal became a real newline, split the literal across
    lines and stopped the entire script -- with nothing in any log, because no
    server ever parses the page it serves.
    """

    def test_the_assets_exist_and_carry_the_features_they_should(self):
        js = (webui.STATIC_DIR / "app.js").read_text()
        html = (webui.STATIC_DIR / "index.html").read_text()
        self.assertIn("askConfirm", js)
        self.assertIn("session id", js)          # the mode-switch explanation
        self.assertIn("MIN_POSE_DRAG_PX", js)    # the pose drag has a floor
        self.assertIn("runExclusive", js)        # map ops are single-flight
        self.assertIn("posehint", html)
        self.assertIn("/static/app.js", html)
        # Bootstrap is vendored, never fetched: the robot has no internet.
        self.assertIn("/static/vendor/bootstrap.min.css", html)
        self.assertNotIn("http://", html)
        self.assertNotIn("https://", html.replace("https://getbootstrap.com", ""))

    def test_assets_are_served_with_their_own_content_types(self):
        for name, kind in (("index.html", "text/html"),
                           ("app.js", "application/javascript"),
                           ("style.css", "text/css"),
                           ("vendor/bootstrap.min.css", "text/css")):
            status, ctype, body = webui._static(name)
            self.assertEqual(status, 200, name)
            self.assertIn(kind, ctype)
            self.assertTrue(body)

    def test_only_the_pages_own_assets_are_readable(self):
        # A fixed set of names, so anything else is a mistake or an attack.
        for name in ("../webui.py", "/etc/passwd", "app.js.bak", ""):
            status, _, _ = webui._static(name)
            self.assertEqual(status, 404, name)


class RangeEndpointTest(unittest.TestCase):
    """A capability the deployment has not bound is absent from the payload,
    not present and empty: the page then has nothing to say about a sensor that
    does not exist, instead of reporting it as perpetually waiting."""

    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), webui._Handler)
        cls.base = "http://127.0.0.1:%d" % cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def get(self):
        with urllib.request.urlopen(self.base + "/api/range", timeout=10) as r:
            return json.loads(r.read())

    def test_scan_only_deployment_reports_only_scan(self):
        self.addCleanup(webui.set_sensor_topics, "", "")
        webui.set_sensor_topics(scan="/scanner_normalized")
        with patch.object(webui, "_overlay_in_map",
                          return_value={"pts": [[1.0, 2.0]], "frame": "l", "stale": False}):
            out = self.get()
        self.assertIn("scan", out)
        self.assertNotIn("cloud", out)
        self.assertEqual(out["bound"], {"scan": "/scanner_normalized"})

    def test_a_deployment_with_no_lidar_reports_nothing_to_draw(self):
        self.addCleanup(webui.set_sensor_topics, "", "")
        webui.set_sensor_topics()
        out = self.get()
        self.assertEqual(out, {"bound": {}})
