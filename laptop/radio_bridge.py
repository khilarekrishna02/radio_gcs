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
        self._last_telem_log = 0.0
        self._last_logged_mode = None
        self._last_logged_armed = None

        

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
        self._log_event(f"Ground radio opened on {self.port} @ {self.baud}")
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
                line = ser.readline()
                if not line:
                    continue

                msg = decode_message(line)
                self.last_rx = time.time()
                typ = msg.get("type")
                payload = msg.get("payload") or {}
                if typ == "RESPONSE":
                    self.requests.resolve(msg.get("request_id"), payload)
                elif typ == "TELEMETRY":
                    first_telem = not self.telemetry.get("connected")
                    self.telemetry = {**payload, "connected": True, "last_update": self.last_rx}
                    if first_telem:
                        self._log_event(f"Drone link connected ({payload.get('mode', 'UNKNOWN')}, {payload.get('armed', 'DISARMED')})")

                    # Log mode or arm state changes
                    cur_mode = payload.get("mode")
                    cur_armed = payload.get("armed")
                    if cur_mode and cur_mode != self._last_logged_mode:
                        if self._last_logged_mode is not None:
                            self._log_event(f"Flight mode changed to {cur_mode}")
                        self._last_logged_mode = cur_mode
                    if cur_armed and cur_armed != self._last_logged_armed:
                        if self._last_logged_armed is not None:
                            self._log_event(f"Vehicle {cur_armed}")
                        self._last_logged_armed = cur_armed

                    # Periodic heartbeat log every 15s to keep feed fresh
                    if self.last_rx - self._last_telem_log > 15.0:
                        self._last_telem_log = self.last_rx
                        lat, lon = payload.get("lat"), payload.get("lon")
                        alt = payload.get("alt")
                        pos_str = f"lat={lat:.5f}, lon={lon:.5f}, alt={alt}m" if lat is not None else "no GPS fix"
                        self._log_event(f"Telemetry alive: {cur_mode} | {pos_str}")

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
            except Exception as e:
                err_msg = str(e)
                # Radio serial links drop occasional RF bytes/packets, leading to partial JSON lines.
                # Skip normal partial-packet parse errors silently without spamming event log.
                if any(x in err_msg for x in (
                    "Expecting value",
                    "Extra data",
                    "Unterminated string",
                    "Expecting ',' delimiter",
                    "Expecting ':' delimiter",
                    "Expecting property name",
                    "Invalid control character"
                )):
                    continue
                self._log_event(f"Radio receive error: {e}")


    
    
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
