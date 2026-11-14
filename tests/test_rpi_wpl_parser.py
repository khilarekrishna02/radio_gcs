import importlib.util
import os
import unittest

JETSON_PROTOCOL = os.path.join(
    os.path.dirname(__file__), "..", "jetson", "protocol.py"
)

spec = importlib.util.spec_from_file_location(
    "jetson_protocol_under_test", JETSON_PROTOCOL
)
jetson_protocol = importlib.util.module_from_spec(spec)
spec.loader.exec_module(jetson_protocol)

parse_qgc_wpl = jetson_protocol.parse_qgc_wpl



class QgcWplParserTests(unittest.TestCase):
    def test_preserves_home_row_and_takeoff_sequence(self):
        wpl = """QGC WPL 110
0\t1\t0\t16\t0\t0\t0\t0\t18.5000000\t73.8000000\t15.00\t1
1\t0\t3\t22\t0\t3\t0\t0\t18.5000000\t73.8000000\t15.00\t1
2\t0\t3\t178\t0\t3\t-1\t0\t0\t0\t0\t1
3\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1
"""
        items = parse_qgc_wpl(wpl)
        self.assertEqual([item["seq"] for item in items], [0, 1, 2, 3])
        self.assertEqual([item["command"] for item in items], [16, 22, 178, 20])
        self.assertEqual(items[0]["current"], 1)
        self.assertEqual(items[1]["frame"], 3)
        self.assertEqual(items[1]["x"], 185000000)

    def test_rejects_non_contiguous_sequences(self):
        wpl = "QGC WPL 110\n2 0 3 22 0 3 0 0 18.5 73.8 15 1\n"
        with self.assertRaises(ValueError):
            parse_qgc_wpl(wpl)


if __name__ == "__main__":
    unittest.main()
