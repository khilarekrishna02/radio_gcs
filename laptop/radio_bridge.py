import threading
import time
import uuid
import serial
from serial.tools import list_ports
import config
from config import RADIO_PORT, RADIO_BAUD, RADIO_TIMEOUT
from protocol import encode_message, decode_message
from request_manager import RequestManager
try:
    from port_detector import detect_radio_port
except ImportError:
    detect_radio_port = None




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
        self.detected_targets = {}
        self.last_rx = None
        self._rxbuf = bytearray()


    @property
    def port_open(self):
        ser = self.serial
        return bool(ser and ser.is_open)

    def _log_event(self, message):
        self.events.append({"time": time.time(), "message": message})
        self.events = self.events[-100:]

    def start(self):
        if self.running:
            return
        self.serial = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.25, write_timeout=3)
        self.running = True
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.serial:
            self.serial.close()

    def _reader(self):
        while self.running:
            # Reopen the ground radio if it was unplugged or never came up.
            ser = self.serial
            if ser is None or not ser.is_open:
                try:
                    port = self.port
                    if getattr(config, "AUTO_DETECT_PORTS", True) and detect_radio_port:
                        detected = detect_radio_port(preferred_port=port)
                        if detected:
                            port = detected
                            self.port = port
                    self.serial = serial.serial_for_url(port, baudrate=self.baud, timeout=0.25, write_timeout=3)
                    self._log_event(f"Ground radio reopened on {port}")
                except Exception:
                    time.sleep(2)
                continue
            try:
                line = self._next_line(ser)
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
                elif typ == "TARGET":
                    try:
                        pid = str(payload["pid"])
                        lat, lon = float(payload["lat"]), float(payload["lon"])
                    except (KeyError, TypeError, ValueError):
                        continue
                    if pid not in self.detected_targets:
                        self._log_event(f"New target {pid} at {lat:.6f}, {lon:.6f}")
                    self.detected_targets[pid] = {"lat": lat, "lon": lon, "time": self.last_rx}
            except serial.SerialException as exc:
                if self.running:
                    self._log_event(f"Ground radio lost: {exc}")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    self.serial = None
                    time.sleep(1)
            except Exception as exc:
                if self.running:
                    self.bad_lines = getattr(self, "bad_lines", 0) + 1
                    if time.time() - getattr(self, "_last_bad_log", 0) > 30:
                        self._last_bad_log = time.time()
                        self.events.append({"time": time.time(), "message": f"Ignored {self.bad_lines} unreadable radio line(s) so far (last: {exc})"})
                        self.events = self.events[-100:]

        def _next_line(self, ser):
          """One complete line (ending in a newline), or b"" if none has arrived yet.
        Bytes are collected across reads, so a line the radio delivers in pieces is not cut in half."""
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            self._rxbuf.extend(chunk)
        end = self._rxbuf.find(b"\n")
        if end < 0:
            if len(self._rxbuf) > 8192:          # runaway data with no newline: start over
                self._rxbuf.clear()
            return b""
        line = bytes(self._rxbuf[:end + 1])
        del self._rxbuf[:end + 1]
        return line
    
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
