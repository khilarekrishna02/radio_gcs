"""SiK radio bridge. This class is run as a thread inside flask_app.py so one process owns the radio port."""
import base64
import hashlib
import json
import os
import threading
import time
import uuid
import serial
import config
from protocol import encode_message, decode_message, parse_qgc_wpl


class RadioBridge:
    def __init__(self, pixhawk):
        self.pixhawk = pixhawk
        self.serial = None
        self.running = threading.Event()
        self.reader_thread = None
        self.telemetry_thread = None
        self.write_lock = threading.Lock()
        self.handler_lock = threading.Lock()
        self.transfer = None
        self.mission_path = None
        self.last_error = None
        self.events = []
        self.response_cache = {}

    def start(self):
        if self.running.is_set():
            return
        os.makedirs(config.MISSION_DIR, exist_ok=True)
        self.serial = serial.Serial(config.RADIO_PORT, config.RADIO_BAUD, timeout=0.25, write_timeout=3)
        self.running.set()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True, name="sik-reader")
        self.telemetry_thread = threading.Thread(target=self._telemetry_sender, daemon=True, name="telemetry-sender")
        self.reader_thread.start()
        self.telemetry_thread.start()

    def stop(self):
        self.running.clear()
        if self.serial:
            self.serial.close()

    def send(self, message_type, payload=None, request_id=None):
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Pi SiK radio is not open")
        raw = encode_message(message_type, payload, request_id)
        with self.write_lock:
            self.serial.write(raw)
            self.serial.flush()

    def _response(self, request_id, ok, message, data=None):
        self.send("RESPONSE", {"ok": bool(ok), "message": str(message), "data": data or {}}, request_id)

    def _cache_response(self, request_id, response):
        if request_id is None:
            return
        key = str(request_id)
        self.response_cache[key] = response
        if len(self.response_cache) > 200:
            self.response_cache.pop(next(iter(self.response_cache)))

    def _event(self, message, level="info", progress=None, total=None):
        payload = {"message": message, "level": level, "time": time.time()}
        if progress is not None: payload["progress"] = progress
        if total is not None: payload["total"] = total
        self.events.append(payload)
        self.events = self.events[-100:]
        try: self.send("EVENT", payload)
        except Exception: pass

    def _reader(self):
        while self.running.is_set():
            try:
                line = self.serial.readline()
                if not line:
                    continue
                msg = decode_message(line)
                request_id = msg.get("request_id")
                payload = msg.get("payload") or {}
                if request_id is not None and str(request_id) in self.response_cache:
                    cached = self.response_cache[str(request_id)]
                    self.send("RESPONSE", cached, request_id)
                    continue
                try:
                    with self.handler_lock:
                        data, response_text = self._handle(msg.get("type"), payload)
                    response = {"ok": True, "message": str(response_text), "data": data or {}}
                    self._cache_response(request_id, response)
                    self.send("RESPONSE", response, request_id)
                except Exception as exc:
                    self._event(f"{msg.get('type')} failed: {exc}", "error")
                    response = {"ok": False, "message": str(exc), "data": {}}
                    self._cache_response(request_id, response)
                    self.send("RESPONSE", response, request_id)
            except Exception as exc:
                if self.running.is_set():
                    self.last_error = str(exc)
                    time.sleep(0.2)

    def _handle(self, kind, p):
        if kind == "PING":
            connected = self.pixhawk.master is not None and self.pixhawk.connected
            return {"pixhawk_connected": connected}, "Pi radio link is responding"
        if kind == "MISSION_BEGIN":
            transfer_id = str(p.get("transfer_id", ""))
            size, chunks = int(p.get("size", -1)), int(p.get("chunks", -1))
            digest = str(p.get("sha256", ""))
            filename = os.path.basename(str(p.get("filename", "mission.waypoints")))
            if not transfer_id or size <= 0 or size > 2_000_000 or chunks <= 0 or len(digest) != 64:
                raise ValueError("Invalid transfer metadata")
            self.transfer = {"id": transfer_id, "size": size, "chunks": chunks, "sha256": digest,
                             "filename": filename, "parts": {}, "received": 0}
            return {"transfer_id": transfer_id, "expected_chunks": chunks}, "Mission transfer initialized"
        if kind == "MISSION_CHUNK":
            t = self.transfer
            if not t or p.get("transfer_id") != t["id"]:
                raise ValueError("No matching active mission transfer")
            index = int(p.get("index", -1))
            if not 0 <= index < t["chunks"]:
                raise ValueError("Chunk index out of range")
            try: chunk = base64.b64decode(p.get("data", ""), validate=True)
            except Exception as exc: raise ValueError("Chunk is not valid base64") from exc
            if len(chunk) > 1024:
                raise ValueError("Chunk too large")
            # Duplicate chunk retry is idempotent; identical duplicate is accepted.
            if index in t["parts"] and t["parts"][index] != chunk:
                raise ValueError("Conflicting duplicate chunk")
            t["parts"][index] = chunk
            return {"index": index, "received_chunks": len(t["parts"]), "total_chunks": t["chunks"]}, f"Chunk {index + 1}/{t['chunks']} accepted"
        if kind == "MISSION_END":
            t = self.transfer
            if not t or p.get("transfer_id") != t["id"]:
                raise ValueError("No matching active mission transfer")
            if len(t["parts"]) != t["chunks"]:
                raise ValueError(f"Missing chunks: received {len(t['parts'])}/{t['chunks']}")
            data = b"".join(t["parts"][i] for i in range(t["chunks"]-0))
            if len(data) != t["size"]:
                raise ValueError(f"File size mismatch: expected {t['size']}, received {len(data)}")
            digest = hashlib.sha256(data).hexdigest()
            if digest != t["sha256"]:
                raise ValueError("SHA-256 checksum mismatch")
            text = data.decode("utf-8")
            # Parse before saving so malformed files are not accepted.
            items = parse_qgc_wpl(text)
            path = os.path.join(config.MISSION_DIR, "received_lawnmower.waypoints")
            with open(path, "wb") as f: f.write(data)
            self.mission_path = path
            self.pixhawk.uploaded = False
            self.transfer = None
            return {"filename": t["filename"], "size": len(data), "sha256": digest,
                    "mission_items": len(items)}, "Mission file reconstructed and checksum verified"
        if kind == "UPLOAD_REQUEST":
            if not self.mission_path or not os.path.isfile(self.mission_path):
                raise RuntimeError("No verified mission file has been received")
            with open(self.mission_path, "r", encoding="utf-8") as f:
                items = parse_qgc_wpl(f.read())
            self._event(f"Uploading {len(items)} mission items to Pixhawk")
            result = self.pixhawk.upload_mission(items, progress=lambda m, n, total: self._event(m, progress=n, total=total))
            return result, "Pixhawk acknowledged mission upload"
        if kind == "ARM_REQUEST":
            if p.get("operator_confirmed") is not True:
                raise PermissionError("Operator confirmation is required")
            result = self.pixhawk.arm_vehicle()
            return result, "Pixhawk armed and heartbeat state confirmed"
        if kind == "START_REQUEST":
            if p.get("operator_confirmed") is not True:
                raise PermissionError("Operator confirmation is required")
            result = self.pixhawk.start_mission()
            return result, "Mission start sequence completed"
        raise ValueError(f"Unsupported message type: {kind}")

    def _telemetry_sender(self):
        while self.running.is_set():
            try:
                self.send("TELEMETRY", dict(self.pixhawk.telemetry))
            except Exception as exc:
                self.last_error = str(exc)
            time.sleep(config.TELEMETRY_PERIOD)
