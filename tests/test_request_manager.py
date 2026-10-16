import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "laptop"))
from request_manager import RequestManager

class RequestManagerTests(unittest.TestCase):
    def test_resolve_and_wait(self):
        manager = RequestManager()
        rid = manager.create()
        manager.resolve(rid, {"ok": True})
        self.assertEqual(manager.wait(rid, 0.1), {"ok": True})

    def test_timeout(self):
        manager = RequestManager()
        rid = manager.create()
        with self.assertRaises(TimeoutError): manager.wait(rid, 0.01)

if __name__ == "__main__": unittest.main()
