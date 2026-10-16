import threading
import uuid


class RequestManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._pending = {}

    def create(self):
        request_id = uuid.uuid4().hex
        event = threading.Event()
        with self._lock:
            self._pending[request_id] = {"event": event, "response": None}
        return request_id

    def rearm(self, request_id):
        """Prepare the same request ID for an idempotent retry."""
        with self._lock:
            self._pending[str(request_id)] = {"event": threading.Event(), "response": None}

    def resolve(self, request_id, response):
        with self._lock:
            item = self._pending.get(str(request_id))
            if item:
                item["response"] = response
                item["event"].set()

    def wait(self, request_id, timeout):
        with self._lock:
            item = self._pending.get(str(request_id))
        if item is None:
            raise KeyError("Unknown request ID")
        if not item["event"].wait(timeout):
            with self._lock:
                self._pending.pop(str(request_id), None)
            raise TimeoutError(f"Timed out waiting for radio response: {request_id}")
        with self._lock:
            item = self._pending.pop(str(request_id), item)
        return item["response"]
