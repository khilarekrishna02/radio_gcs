"""Offline tests for message framing and WPL parser; no radios or flight controller required."""
import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "laptop"))
from protocol import encode_message, decode_message
from mission_planner import parse_qgc_wpl

SAMPLE = """QGC WPL 110
0\t1\t0\t16\t0\t0\t0\t0\t18.0000000\t73.0000000\t15.00\t1
1\t0\t3\t22\t0\t3\t0\t0\t18.0000000\t73.0000000\t15.00\t1
2\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1
"""

class SerialLinkTests(unittest.TestCase):
    def test_line_delimited_message(self):
        wire = encode_message("MISSION_CHUNK", {"index": 0, "data": "YWJj"}, "req-1")
        self.assertTrue(wire.endswith(b"\n"))
        self.assertEqual(decode_message(wire)["payload"]["data"], "YWJj")

    def test_qgc_home_placeholder_skipped(self):
        items = parse_qgc_wpl(SAMPLE)
        self.assertEqual(len(items), 2)
        self.assertEqual(items[0]["command"], 22)

if __name__ == "__main__": unittest.main()
