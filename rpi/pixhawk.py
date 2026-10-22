"""Pixhawk serial connection and MAVLink mission operations. One process owns this connection."""
import threading
import time
from pymavlink import mavutil
import config


class PixhawkController:
    def __init__(self):
        self.master = None
        self.lock = threading.RLock()
        self.connected = False
        self.last_error = None
        self.telemetry = {"lat": None, "lon": None, "alt": None, "mode": None,
                          "status": "Waiting for Pixhawk", "groundspeed": None,
                          "lidar_reading": None, "armed": "UNKNOWN"}
        self.uploaded = False
        self.mission_item_count = 0

    def connect(self):
        if self.master is not None:
            return True
        port = config.PIXHAWK_PORT
        baud = config.PIXHAWK_BAUD
        conn = None
        try:
            if config.PIXHAWK_CONNECTION_MODE == "udp":
                conn = mavutil.mavlink_connection(port)
            else:
                conn = mavutil.mavlink_connection(port, baud=baud, autoreconnect=True)
            hb = conn.wait_heartbeat(timeout=8)
            if hb is None:
                conn.close()
                raise TimeoutError(f"No Pixhawk heartbeat on {port}")
            self.master = conn
            try:
                self.master._autopilot_component_id = hb.get_srcComponent()
            except Exception:
                pass
            self.connected = True
            self.telemetry["mode"] = getattr(conn, "flightmode", "UNKNOWN")
            self.telemetry["status"] = f"Pixhawk connected on {port}"
            try:
                conn.mav.request_data_stream_send(conn.target_system, conn.target_component,
                                                 mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            except Exception:
                pass
            return True
        except Exception as exc:
            self.last_error = str(exc)
            self.connected = False
            if conn is not None:
                try: conn.close()
                except Exception: pass
            raise

    def ensure_connected(self):
        if self.master is None or not self.connected:
            self.connect()

    def telemetry_loop(self, running_event):
        while running_event.is_set():
            if self.master is None:
                try:
                    self.connect()
                except Exception as exc:
                    self.telemetry["status"] = f"Pixhawk reconnecting: {exc}"
                    time.sleep(3)
                    continue
            try:
                with self.lock:
                    msg = self.master.recv_match(blocking=True, timeout=1)
                    if msg is None:
                        continue
                    kind = msg.get_type()
                    if kind == "GLOBAL_POSITION_INT":
                        self.telemetry["lat"] = msg.lat / 1e7
                        self.telemetry["lon"] = msg.lon / 1e7
                        self.telemetry["alt"] = msg.relative_alt / 1000.0
                        self.telemetry["mode"] = getattr(self.master, "flightmode", "UNKNOWN")
                        self.telemetry["armed"] = "ARMED" if self.master.motors_armed() else "DISARMED"
                    elif kind == "STATUSTEXT":
                        self.telemetry["status"] = msg.text
                    elif kind == "VFR_HUD":
                        self.telemetry["groundspeed"] = msg.groundspeed
                    elif kind == "DISTANCE_SENSOR" and config.LIDAR_ENABLED:
                        if hasattr(msg, "current_distance"):
                            self.telemetry["lidar_reading"] = msg.current_distance
            except Exception as exc:
                self.last_error = str(exc)
                self.telemetry["status"] = f"Telemetry error: {exc}"
                self.connected = False
                try:
                    if self.master: self.master.close()
                except Exception: pass
                self.master = None
                time.sleep(2)

    def parse_waypoints(self, text):
        from protocol import parse_qgc_wpl
        return parse_qgc_wpl(text)

    def _mission_type_kwargs(self):
        return {"mission_type": mavutil.mavlink.MAV_MISSION_TYPE_MISSION}

    @staticmethod
    def _command_uses_global_position(command):
        """Return whether a mission command's x/y/z fields are geographic position."""
        # NAV commands whose x/y fields describe a location. Non-positional commands
        # such as DO_CHANGE_SPEED and RTL may be normalized to frame 0 by ArduPilot.
        return int(command) in {
            16,  # MAV_CMD_NAV_WAYPOINT
            17,  # MAV_CMD_NAV_LOITER_UNLIM
            18,  # MAV_CMD_NAV_LOITER_TURNS
            19,  # MAV_CMD_NAV_LOITER_TIME
            21,  # MAV_CMD_NAV_LAND
            22,  # MAV_CMD_NAV_TAKEOFF
            82,  # MAV_CMD_NAV_SPLINE_WAYPOINT
            84,  # MAV_CMD_NAV_LOITER_TO_ALT
            85,  # MAV_CMD_NAV_GUIDED_ENABLE (x/y not geographic in all implementations; handled conservatively)
            189, # MAV_CMD_DO_LAND_START (frame may be relevant on some firmwares)
        }

    @staticmethod
    def _int_frame_for_item(frame, command):
        """Use INT-specific global frame IDs for position-bearing MISSION_ITEM_INT."""
        frame = int(frame)
        if not PixhawkController._command_uses_global_position(command):
            return frame
        mapping = {
            mavutil.mavlink.MAV_FRAME_GLOBAL: mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT: mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT: mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
        }
        return int(mapping.get(frame, frame))

    def upload_mission(self, items, progress=None):
        """Upload QGC WPL rows with request-driven MAVLink logic.

        Preserve all parsed rows, including TAKEOFF (command 22). Success requires
        the final accepted MISSION_ACK; no post-upload readback comparison is done.
        """
        if not items:
            raise ValueError("Mission contains no MAVLink items")
        self.ensure_connected()
        master = self.master
        total = len(items)
        target_system = master.target_system
        target_component = getattr(master, "_autopilot_component_id", None) or master.target_component

        with self.lock:
            self.uploaded = False
            self.mission_item_count = 0
            if progress:
                progress("Clearing previous mission", 0, total)
            try:
                master.mav.mission_clear_all_send(
                    target_system, target_component,
                    mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )
            except TypeError:
                master.mav.mission_clear_all_send(target_system, target_component)

            # Match the known-good standalone uploader's clear-then-upload flow.
            # Drain a delayed clear ACK before starting the new transaction so it
            # cannot be mistaken for the upload's final ACK.
            time.sleep(2)
            while master.recv_match(type="MISSION_ACK", blocking=False) is not None:
                pass

            try:
                master.mav.mission_count_send(
                    target_system, target_component, total,
                    mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )
            except TypeError:
                master.mav.mission_count_send(target_system, target_component, total)
            sent = set()
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                msg = master.recv_match(
                    type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                    blocking=True, timeout=2
                )
                if msg is None:
                    continue
                msg_type = msg.get_type()
                if msg_type == "MISSION_ACK":
                    if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED or len(sent) != total:
                        raise RuntimeError(
                            f"Mission rejected/incomplete: ACK={msg.type}, sent={len(sent)}/{total}"
                        )
                    break

                seq = int(msg.seq)
                if not 0 <= seq < total:
                    raise RuntimeError(f"Pixhawk requested invalid mission sequence {seq}")
                item = items[seq]
                print(
                    f"Mission item requested: seq={seq}, command={item['command']}, "
                    f"frame={item['frame']}, altitude={item['z']}",
                    flush=True,
                )
                if msg_type == "MISSION_REQUEST_INT":
                    wire_frame = self._int_frame_for_item(item["frame"], item["command"])
                    master.mav.mission_item_int_send(
                        target_system, target_component, seq,
                        wire_frame, item["command"], item["current"], item["autocontinue"],
                        item["param1"], item["param2"], item["param3"], item["param4"],
                        item["x"], item["y"], item["z"],
                        mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                    )
                else:
                    x, y = item["x"], item["y"]
                    if item["frame"] in (
                        mavutil.mavlink.MAV_FRAME_GLOBAL,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
                    ):
                        x, y = x / 1e7, y / 1e7
                    master.mav.mission_item_send(
                        target_system, target_component, seq,
                        item["frame"], item["command"], item["current"], item["autocontinue"],
                        item["param1"], item["param2"], item["param3"], item["param4"],
                        x, y, item["z"],
                        mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                    )
                sent.add(seq)
                if progress:
                    progress(f"Uploading mission item {seq + 1}/{total}", len(sent), total)
            else:
                raise TimeoutError(f"Mission upload timed out; sent {len(sent)}/{total} rows")

            if progress:
                progress("Mission accepted by Pixhawk", total, total)

            self.uploaded = True
            self.mission_item_count = total
            self.telemetry["status"] = f"Mission upload accepted ({total} items)"
            commands = [int(item["command"]) for item in items]
            print(f"Pixhawk accepted mission upload: {total} items; commands={commands}", flush=True)
            if 22 not in commands:
                print("WARNING: Uploaded mission contains no TAKEOFF command (22)", flush=True)
            return {
                "uploaded_items": total,
                "mission_ack": "MAV_MISSION_ACCEPTED",
                "readback_verified": False,
                "commands": commands,
                "takeoff_present": 22 in commands,
            }

    def _wait_mode(self, expected, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
                current = getattr(self.master, "flightmode", "UNKNOWN")
            if current == expected:
                return
        raise TimeoutError(f"Pixhawk did not confirm {expected} mode")

    def arm_vehicle(self):
        """Request arming, report MAVLink ACK/status text, and verify heartbeat state."""
        self.ensure_connected()
        master = self.master

        with self.lock:
            if master.motors_armed():
                self.telemetry["armed"] = "ARMED"
                return {"armed": True, "status": "ALREADY_ARMED"}

            # Use the autopilot target learned from its heartbeat.
            target_system = master.target_system
            target_component = master.target_component
            master.mav.command_long_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,  # param1: arm
                0, 0, 0, 0, 0, 0,
            )

        deadline = time.monotonic() + 10
        last_status_text = None
        ack_received = False

        while time.monotonic() < deadline:
            with self.lock:
                msg = master.recv_match(
                    type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"],
                    blocking=True,
                    timeout=1,
                )
                armed = master.motors_armed()

            if msg is not None:
                kind = msg.get_type()
                if kind == "STATUSTEXT":
                    last_status_text = str(msg.text).strip("\x00")
                    self.telemetry["status"] = last_status_text
                    print(f"Pixhawk STATUSTEXT: {last_status_text}")
                elif kind == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    ack_received = True
                    print(f"Pixhawk ARM COMMAND_ACK: {msg.to_dict()}")
                    if msg.result not in (
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                    ):
                        detail = f"Pixhawk rejected ARM command (result={msg.result})"
                        if last_status_text:
                            detail += f": {last_status_text}"
                        raise RuntimeError(detail)

            if armed:
                self.telemetry["armed"] = "ARMED"
                self.telemetry["status"] = "ARMED"
                return {
                    "armed": True,
                    "status": "ARMED",
                    "command_ack_received": ack_received,
                    "statustext": last_status_text,
                }

        detail = "Pixhawk did not confirm arming within 10 seconds."
        if last_status_text:
            detail += f" Last STATUSTEXT: {last_status_text}"
        if not ack_received:
            detail += " No ARM COMMAND_ACK was received."
        raise TimeoutError(detail)

    def disarm_vehicle(self):
        """Request normal disarming and wait for heartbeat state confirmation.

        This intentionally does not use ArduPilot force-disarm: stopping motors
        while airborne can cause a crash. Use after landing or for secured testing.
        """
        self.ensure_connected()
        master = self.master
        with self.lock:
            if not master.motors_armed():
                self.telemetry["armed"] = "DISARMED"
                self.telemetry["status"] = "DISARMED"
                return {"armed": False, "status": "ALREADY_DISARMED"}
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0,
            )

        deadline = time.monotonic() + 10
        last_status_text = None
        ack_received = False
        while time.monotonic() < deadline:
            with self.lock:
                msg = master.recv_match(
                    type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"],
                    blocking=True, timeout=1,
                )
                armed = master.motors_armed()
            if msg is not None:
                kind = msg.get_type()
                if kind == "STATUSTEXT":
                    last_status_text = str(msg.text).strip("\x00")
                    self.telemetry["status"] = last_status_text
                    print(f"Pixhawk STATUSTEXT: {last_status_text}")
                elif kind == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    ack_received = True
                    print(f"Pixhawk DISARM COMMAND_ACK: {msg.to_dict()}")
                    if msg.result not in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                          mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
                        detail = f"Pixhawk rejected DISARM command (result={msg.result})"
                        if last_status_text:
                            detail += f": {last_status_text}"
                        raise RuntimeError(detail)
            if not armed:
                self.telemetry["armed"] = "DISARMED"
                self.telemetry["status"] = "DISARMED"
                return {"armed": False, "status": "DISARMED",
                        "command_ack_received": ack_received, "statustext": last_status_text}
        detail = "Pixhawk did not confirm disarming within 10 seconds."
        if last_status_text:
            detail += f" Last STATUSTEXT: {last_status_text}"
        if not ack_received:
            detail += " No DISARM COMMAND_ACK was received."
        raise TimeoutError(detail)

    def start_mission(self):
        self.ensure_connected()
        if not self.uploaded:
            raise RuntimeError("No mission has been acknowledged by Pixhawk")
        if self.telemetry.get("lat") is None or self.telemetry.get("lon") is None:
            raise RuntimeError("GPS position is not available; refusing to start")
        master = self.master
        with self.lock:
            modes = master.mode_mapping() or {}
            if "GUIDED" not in modes or "AUTO" not in modes:
                raise RuntimeError("GUIDED/AUTO modes are not available in the current firmware")
            master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, modes["GUIDED"])
        self._wait_mode("GUIDED")
        self.arm_vehicle()
        with self.lock:
            master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, modes["AUTO"])
        self._wait_mode("AUTO")
        with self.lock:
            master.mav.command_long_send(master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
        return {"sequence": "GUIDED -> ARM confirmed -> AUTO confirmed -> MISSION_START sent",
                "note": "Existing mission commands, including takeoff, are preserved."}
