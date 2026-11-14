#!/usr/bin/env python3
"""Stand-in for the Jetson, for testing at home.

Runs the REAL jetson/radio_bridge.py (the same code that runs on the drone) with a
fake Pixhawk and a fake camera/vision, so the laptop GCS and the web page can be
tested end to end with no hardware.

    python fake_jetson.py        (start virtual_radio.py first)

Press "Execute mission" on the page and it pretends the mission started: the
drone climbs to 15 m and flies a circle, and every few seconds a fake TARGET is
sent so you can see markers and distances. Type  land  to stop, quit to exit.
Nothing here touches real hardware.
"""
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))                       # repo root: laptop/ jetson/ tests/
JETSON = next((os.path.join(ROOT, d) for d in ("jetson", "rpi") if os.path.isdir(os.path.join(ROOT, d))), None)
if JETSON is None:
    sys.exit("Cannot find the jetson/ folder next to tests/")
os.environ.setdefault("GCS_RADIO_PORT", "socket://127.0.0.1:9002")
os.environ.setdefault("GCS_MISSION_DIR", os.path.join(HERE, "_fake_jetson_missions"))
os.environ.setdefault("GCS_TELEMETRY_PERIOD", "1.0")
if not os.path.isfile(os.path.join(JETSON, "protocol.py")):
    sys.exit("jetson/protocol.py is missing - the Jetson bridge needs the full protocol.py next to it")
sys.path.insert(0, JETSON)
import jetson.radio_bridge # noqa: E402  (the real Jetson bridge)

HOME_LAT = float(os.getenv("FAKE_LAT", "18.5293"))
HOME_LON = float(os.getenv("FAKE_LON", "73.8566"))
M_PER_DEG = 111320.0
CIRCLE_R, SPEED, CRUISE_ALT = 40.0, 3.0, 15.0
# fake people, metres (east, north) from home
FAKE_TARGETS = {1: (25, 0), 2: (0, 40), 3: (-30, -10)}


def offset(e, n):
    return HOME_LAT + n / M_PER_DEG, HOME_LON + e / (M_PER_DEG * math.cos(math.radians(HOME_LAT)))


class FakePixhawk:
    master = object()
    connected = True

    def __init__(self):
        self.uploaded = False
        self.mission_item_count = 0
        self.flying = False
        self.angle = 0.0
        self.telemetry = {"lat": HOME_LAT, "lon": HOME_LON, "alt": 0.0, "groundspeed": 0.0,
                          "mode": "STABILIZE", "armed": "DISARMED", "status": "FAKE JETSON (home test)",
                          "lidar_reading": 0}

    def upload_mission(self, items, progress=None):
        n = len(items)
        print(f"[fake pixhawk] mission upload: {n} items (first: seq {items[0]['seq']} cmd {items[0]['command']})")
        for i in range(1, 5):
            if progress:
                progress(f"Uploading mission item {i * n // 4}/{n}", i * n // 4, n)
            time.sleep(0.2)
        self.uploaded, self.mission_item_count = True, n
        return {"uploaded_items": n, "mission_ack": "MAV_MISSION_ACCEPTED (fake)"}

    def arm_vehicle(self):
        self.telemetry["armed"] = "ARMED"
        print("[fake pixhawk] ARMED")
        return {"armed": True}

    def disarm_vehicle(self):
        self.telemetry["armed"] = "DISARMED"
        self.flying = False
        print("[fake pixhawk] DISARMED")
        return {"armed": False}

    def start_mission(self):
        if not self.uploaded:
            raise RuntimeError("No mission uploaded to the Pixhawk")
        self.telemetry.update(armed="ARMED", mode="AUTO")
        self.flying = True
        print("[fake pixhawk] GUIDED -> ARM -> AUTO -> mission start")
        return {"sequence": "GUIDED -> ARM -> AUTO -> MISSION_START (fake)"}

    def land(self):
        self.flying = False
        self.telemetry.update(mode="LAND", groundspeed=0.0)

    def sim_loop(self):
        dt = 0.5
        while True:
            t = self.telemetry
            if self.flying:
                t["alt"] = min(CRUISE_ALT, t["alt"] + 1.0 * dt)
                self.angle += SPEED / CIRCLE_R * dt
                t["lat"], t["lon"] = offset(CIRCLE_R * math.sin(self.angle), CIRCLE_R * math.cos(self.angle))
                t["groundspeed"] = SPEED
            else:
                t["alt"] = max(0.0, t["alt"] - 1.0 * dt)
                if t["alt"] == 0.0 and t["mode"] == "LAND":
                    t["armed"], t["mode"] = "DISARMED", "STABILIZE"
            t["lidar_reading"] = int(t["alt"] * 100)       # cm, like the real sensor
            time.sleep(dt)


class FakeVision:
    camera_running = True
    model_name = "bestest.pt"

    def start_camera(self):
        self.camera_running = True
        print("[fake vision] camera STARTED")

    def stop_camera(self):
        self.camera_running = False
        print("[fake vision] camera STOPPED")

    def load_model(self, name):
        self.model_name = name
        print(f"[fake vision] model switched to {name}")


def target_loop(px, rb):
    pid = 0
    while True:
        time.sleep(6)
        if px.flying:
            pid = pid % len(FAKE_TARGETS) + 1
            lat, lon = offset(*FAKE_TARGETS[pid])
            if rb.send_target(pid, lat, lon):
                print(f"[fake vision] TARGET {pid} sent at {lat:.6f}, {lon:.6f}")


def main():
    px, vis = FakePixhawk(), FakeVision()
    rb = radio_bridge.RadioBridge(px)
    rb.vision_thread = vis
    rb.start()
    threading.Thread(target=px.sim_loop, daemon=True).start()
    threading.Thread(target=target_loop, args=(px, rb), daemon=True).start()
    print(f"[fake jetson] real radio_bridge.py running on {os.environ['GCS_RADIO_PORT']}; fake drone at {HOME_LAT}, {HOME_LON}")
    print("[fake jetson] commands: land | quit")
    while True:
        try:
            cmd = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd == "land":
            px.land()
            print("[fake pixhawk] landing")
        if cmd == "quit":
            break
    rb.stop()


if __name__ == "__main__":
    main()
