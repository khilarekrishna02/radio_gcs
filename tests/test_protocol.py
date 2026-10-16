import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "laptop"))
from protocol import encode_message, decode_message

class ProtocolTests(unittest.TestCase):
    def test_round_trip(self):
        msg = decode_message(encode_message("PING", {"x": 1}, "abc").strip())
        self.assertEqual(msg["type"], "PING")
        self.assertEqual(msg["request_id"], "abc")
        self.assertEqual(msg["payload"]["x"], 1)

    def test_bad_version_rejected(self):
        with self.assertRaises(ValueError):
            decode_message('{"v":2,"type":"PING"}')

if __name__ == "__main__": unittest.main()
