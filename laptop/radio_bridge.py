import threading
import time
import uuid
import serial
from serial.tools import list_ports
from config import RADIO_PORT, RADIO_BAUD, RADIO_TIMEOUT
from protocol import encode_message, decode_message
from request_manager import RequestManager


class RadioBridge:
    """Owns the laptop ground-radio serial port. Do not open COM13 elsewhere."""
    def __init__(self, port=RADIO_PORT, baud=RADIO_BAUD):
        self.port, self.baud = port, baud
        self.serial = None
        self.requests = RequestManager()
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.telemetry = {"connected": False}
        self.events = []
        self.last_rx = None

    def start(self):
        if self.running:
            return
        self.serial = serial.Serial(self.port, self.baud, timeout=0.25, write_timeout=3)
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.serial:
            self.serial.close()

    def _reader(self):
        while self.running:
            try:
                line = self.serial.readline()
                if not line:
                    continue
                msg = decode_message(line)
                self.last_rx = time.time()
                typ = msg.get("type")
                payload = msg.get("payload") or {}
                if typ == "RESPONSE":
                    self.requests.resolve(msg.get("request_id"), payload)
                elif typ == "TELEMETRY":
                    self.telemetry = {**payload, "connected": True, "last_update": self.last_rx}
                elif typ == "EVENT":
                    self.events.append({"time": self.last_rx, **payload})
                    self.events = self.events[-100:]
            except Exception as exc:
                if self.running:
                    self.events.append({"time": time.time(), "message": f"Radio receive error: {exc}"})
                    self.events = self.events[-100:]

    def _write(self, raw):
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Ground radio is not open")
        with self.lock:
            self.serial.write(raw)
            self.serial.flush()

    def request(self, message_type, payload=None, timeout=None, attempts=3):
        request_id = self.requests.create()
        wait_seconds = timeout or RADIO_TIMEOUT
        last_error = None
        for attempt in range(attempts):
            if attempt:
                self.requests.rearm(request_id)
            self._write(encode_message(message_type, payload or {}, request_id))
            try:
                response = self.requests.wait(request_id, wait_seconds)
                if not response or not response.get("ok", False):
                    raise RuntimeError((response or {}).get("message", "Pi returned an unsuccessful response"))
                return response
            except TimeoutError as exc:
                last_error = exc
        raise last_error or TimeoutError("Radio request failed")

    def send_mission_file(self, path, chunk_bytes=360):
        import base64, hashlib, os
        with open(path, "rb") as f:
            data = f.read()
        transfer_id = uuid.uuid4().hex
        digest = hashlib.sha256(data).hexdigest()
        chunks = [data[i:i + chunk_bytes] for i in range(0, len(data), chunk_bytes)]
        self.request("MISSION_BEGIN", {"transfer_id": transfer_id, "filename": os.path.basename(path),
                                      "size": len(data), "sha256": digest, "chunks": len(chunks)}, timeout=15)
        for index, chunk in enumerate(chunks):
            self.request("MISSION_CHUNK", {"transfer_id": transfer_id, "index": index,
                                           "data": base64.b64encode(chunk).decode("ascii")}, timeout=15)
        result = self.request("MISSION_END", {"transfer_id": transfer_id}, timeout=20)
        return {"transfer_id": transfer_id, "sha256": digest, "size": len(data), **result.get("data", {})}


def list_serial_ports():
    return [{"device": p.device, "description": p.description} for p in list_ports.comports()]
