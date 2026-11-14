
import base64
import hashlib
import os
import threading
import time

import serial
import config
try:
    from port_detector import detect_radio_port
except ImportError:
    detect_radio_port = None

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

    # --------------------------------------------------
    # START / STOP
    # --------------------------------------------------

    def start(self):
        if self.running.is_set():
            return

        os.makedirs(config.MISSION_DIR, exist_ok=True)

        port = config.RADIO_PORT
        if getattr(config, "AUTO_DETECT_PORTS", True) and detect_radio_port:
            exclude = None
            if self.pixhawk and getattr(self.pixhawk, "master", None):
                exclude = getattr(self.pixhawk.master, "address", None)
            detected = detect_radio_port(exclude_port=exclude, preferred_port=port)
            if detected:
                port = detected

        self.serial = serial.Serial(
            port,
            config.RADIO_BAUD,
            timeout=0.25,
            write_timeout=3
        )


        self.running.set()

        self.reader_thread = threading.Thread(
            target=self._reader,
            daemon=True,
            name="drone2-radio-reader"
        )

        self.telemetry_thread = threading.Thread(
            target=self._telemetry_sender,
            daemon=True,
            name="drone2-telemetry-sender"
        )

        self.reader_thread.start()
        self.telemetry_thread.start()

        print(f"Drone 2 SiK radio connected: {config.RADIO_PORT}")

    def stop(self):
        self.running.clear()

        if self.serial and self.serial.is_open:
            self.serial.close()

    # --------------------------------------------------
    # MESSAGE SENDING
    # --------------------------------------------------

    def send(self, message_type, payload=None, request_id=None):
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Drone 2 SiK radio is not open")

        raw = encode_message(
            message_type,
            payload,
            request_id
        )

        with self.write_lock:
            self.serial.write(raw)
            self.serial.flush()

    def _send_response(self, request_id, ok, message, data=None):
        response = {
            "ok": bool(ok),
            "message": str(message),
            "data": data or {}
        }

        self._cache_response(request_id, response)

        self.send("RESPONSE", response, request_id)

    def _cache_response(self, request_id, response):
        if request_id is None:
            return

        self.response_cache[str(request_id)] = response

        if len(self.response_cache) > 200:
            oldest = next(iter(self.response_cache))
            self.response_cache.pop(oldest)

    # --------------------------------------------------
    # EVENTS
    # --------------------------------------------------

    def _event(self, message, level="info", progress=None, total=None):
        event = {
            "message": message,
            "level": level,
            "time": time.time()
        }

        if progress is not None:
            event["progress"] = progress

        if total is not None:
            event["total"] = total

        self.events.append(event)
        self.events = self.events[-100:]

        try:
            self.send("EVENT", event)
        except Exception:
            pass

    # --------------------------------------------------
    # RADIO READER
    # --------------------------------------------------

    def _reader(self):
        while self.running.is_set():
            try:
                line = self.serial.readline()

                if not line:
                    continue

                msg = decode_message(line)

                request_id = msg.get("request_id")
                message_type = msg.get("type")
                payload = msg.get("payload") or {}

                # Handle duplicate requests safely.
                if (
                    request_id is not None
                    and str(request_id) in self.response_cache
                ):
                    self.send(
                        "RESPONSE",
                        self.response_cache[str(request_id)],
                        request_id
                    )
                    continue

                try:
                    with self.handler_lock:
                        data, response_text = self._handle(
                            message_type,
                            payload
                        )

                    self._send_response(
                        request_id,
                        True,
                        response_text,
                        data
                    )

                except Exception as exc:
                    self.last_error = str(exc)

                    self._event(
                        f"{message_type} failed: {exc}",
                        level="error"
                    )

                    self._send_response(
                        request_id,
                        False,
                        str(exc)
                    )

            except Exception as exc:
                if self.running.is_set():
                    self.last_error = str(exc)
                    time.sleep(0.2)

    # --------------------------------------------------
    # COMMAND HANDLER
    # --------------------------------------------------

    def _handle(self, kind, payload):

        # Laptop connectivity check
        if kind == "PING":
            connected = (
                self.pixhawk.master is not None
                and self.pixhawk.connected
            )

            return {
                "pixhawk_connected": connected
            }, "Drone 2 radio link responding"

        # ----------------------------------------------
        # MISSION FILE TRANSFER
        # ----------------------------------------------

        if kind == "MISSION_BEGIN":

            transfer_id = str(payload.get("transfer_id", ""))
            size = int(payload.get("size", -1))
            chunks = int(payload.get("chunks", -1))
            digest = str(payload.get("sha256", ""))

            filename = os.path.basename(
                str(payload.get("filename", "mission.waypoints"))
            )

            if (
                not transfer_id
                or size <= 0
                or size > 2_000_000
                or chunks <= 0
                or chunks > 10000
                or len(digest) != 64
            ):
                raise ValueError("Invalid mission transfer metadata")

            self.transfer = {
                "id": transfer_id,
                "size": size,
                "chunks": chunks,
                "sha256": digest,
                "filename": filename,
                "parts": {}
            }

            return {
                "transfer_id": transfer_id,
                "expected_chunks": chunks
            }, "Mission transfer initialized"

        # ----------------------------------------------

        if kind == "MISSION_CHUNK":

            transfer = self.transfer

            if (
                not transfer
                or payload.get("transfer_id") != transfer["id"]
            ):
                raise ValueError("No matching mission transfer")

            index = int(payload.get("index", -1))

            if not 0 <= index < transfer["chunks"]:
                raise ValueError("Invalid chunk index")

            try:
                chunk = base64.b64decode(
                    payload.get("data", ""),
                    validate=True
                )
            except Exception as exc:
                raise ValueError("Invalid base64 chunk") from exc

            if len(chunk) > 1024:
                raise ValueError("Chunk exceeds 1024 bytes")

            existing = transfer["parts"].get(index)

            if existing is not None and existing != chunk:
                raise ValueError("Conflicting duplicate chunk")

            transfer["parts"][index] = chunk

            return {
                "index": index,
                "received_chunks": len(transfer["parts"]),
                "total_chunks": transfer["chunks"]
            }, f"Chunk {index + 1}/{transfer['chunks']} received"

        # ----------------------------------------------

        if kind == "MISSION_END":

            transfer = self.transfer

            if (
                not transfer
                or payload.get("transfer_id") != transfer["id"]
            ):
                raise ValueError("No matching mission transfer")

            if len(transfer["parts"]) != transfer["chunks"]:
                raise ValueError("Mission transfer is incomplete")

            data = b"".join(
                transfer["parts"][i]
                for i in range(transfer["chunks"])
            )

            if len(data) != transfer["size"]:
                raise ValueError("Mission file size mismatch")

            digest = hashlib.sha256(data).hexdigest()

            if digest != transfer["sha256"]:
                raise ValueError("Mission checksum mismatch")

            mission_text = data.decode("utf-8")

            # Validate mission before accepting it.
            items = parse_qgc_wpl(mission_text)

            path = os.path.join(
                config.MISSION_DIR,
                "drone2_mission.waypoints"
            )

            temp_path = path + ".tmp"

            with open(temp_path, "wb") as file:
                file.write(data)

            os.replace(temp_path, path)

            self.mission_path = path
            self.pixhawk.uploaded = False
            self.transfer = None

            return {
                "filename": transfer["filename"],
                "size": len(data),
                "sha256": digest,
                "mission_items": len(items)
            }, "Mission received and verified"

        # ----------------------------------------------
        # PIXHAWK MISSION UPLOAD
        # ----------------------------------------------

        if kind == "UPLOAD_REQUEST":

            if (
                not self.mission_path
                or not os.path.isfile(self.mission_path)
            ):
                raise RuntimeError("No verified mission received")

            with open(
                self.mission_path,
                "r",
                encoding="utf-8"
            ) as file:
                items = parse_qgc_wpl(file.read())

            self._event(
                f"Uploading {len(items)} mission items to Pixhawk"
            )

            result = self.pixhawk.upload_mission(
                items,
                progress=lambda message, current, total:
                    self._event(
                        message,
                        progress=current,
                        total=total
                    )
            )

            return result, "Pixhawk mission upload acknowledged"

        # ----------------------------------------------
        # ARM
        # ----------------------------------------------

        if kind == "ARM_REQUEST":

            if payload.get("operator_confirmed") is not True:
                raise PermissionError("Operator confirmation required")

            result = self.pixhawk.arm_vehicle()

            return result, "Pixhawk armed"

        # ----------------------------------------------
        # DISARM
        # ----------------------------------------------

        if kind == "DISARM_REQUEST":

            if payload.get("operator_confirmed") is not True:
                raise PermissionError("Operator confirmation required")

            result = self.pixhawk.disarm_vehicle()

            return result, "Pixhawk disarmed"

        # ----------------------------------------------
        # START MISSION
        # ----------------------------------------------

        if kind == "START_REQUEST":

            if payload.get("operator_confirmed") is not True:
                raise PermissionError("Operator confirmation required")

            result = self.pixhawk.start_mission()

            return result, "Mission start sequence completed"

        raise ValueError(f"Unsupported message type: {kind}")

    # --------------------------------------------------
    # TELEMETRY TO LAPTOP
    # --------------------------------------------------

    def _telemetry_sender(self):

        while self.running.is_set():

            try:
                telemetry = dict(self.pixhawk.telemetry)

                self.send(
                    "TELEMETRY",
                    telemetry
                )

            except Exception as exc:
                self.last_error = str(exc)

            time.sleep(config.TELEMETRY_PERIOD)
