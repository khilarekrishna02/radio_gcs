"""Checks that the Jetson-side files work TOGETHER, on any laptop, with no Jetson/camera/GPU/Pixhawk.

Run from the project folder:   python tests/test_jetson_wiring.py

torch, cv2, ultralytics, pyrealsense2 and pymavlink are replaced by tiny fakes, so only numpy and pandas
must be installed. It checks: importing vision_geotag has no side effects, PixhawkController's buffers
(cm->m, range filter, staleness), the geotag geometry through Geotagger.process_frame, camera start/stop,
and main_surveillance.surveillance_loop sending TARGET messages."""
import sys, types, time, threading, contextlib, math, os, io, tempfile
import numpy as np

try:                                   # Windows consoles: keep the arrows/degree signs the drone code prints
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
JETSON = next((os.path.join(ROOT, d) for d in ("jetson", "rpi") if os.path.isdir(os.path.join(ROOT, d))), None)
if JETSON is None:
    sys.exit("Cannot find the jetson/ folder next to tests/")
sys.path.insert(0, JETSON)
WORK = tempfile.mkdtemp(prefix="wiring_test_")
os.environ["VISION_OUTPUT_DIR"] = os.path.join(WORK, "frames")
os.environ["GCS_MISSION_DIR"] = os.path.join(WORK, "missions")

# ---------- stubs ----------
created = {"pipelines": 0}
torch = types.ModuleType("torch")
torch.cuda = types.SimpleNamespace(is_available=lambda: False)
@contextlib.contextmanager
def no_grad(): yield
torch.no_grad = no_grad
sys.modules["torch"] = torch

cv2 = types.ModuleType("cv2")
cv2.FONT_HERSHEY_SIMPLEX = 0
cv2.rectangle = lambda *a, **k: None
cv2.putText = lambda *a, **k: None
cv2.imwrite = lambda *a, **k: True
sys.modules["cv2"] = cv2

BOXES = []   # what the fake detector returns on each call
class FakeBox:
    def __init__(s, x1, y1, x2, y2, tid): s.cls=[0]; s.xyxy=[[x1,y1,x2,y2]]; s.id=[tid]
class FakeYOLO:
    device = "cpu"
    def __init__(s, path): s.path = path
    def to(s, d): pass
    def track(s, img, **kw):
        assert kw["half"] is False, "half must be off on CPU"
        return [types.SimpleNamespace(boxes=list(BOXES))]
ul = types.ModuleType("ultralytics"); ul.YOLO = FakeYOLO; sys.modules["ultralytics"] = ul

class FakeFrame:
    def __init__(s, ms): s.ms = ms
    def get_color_frame(s): return s
    def get_data(s): return np.zeros((720, 1280, 3), np.uint8)
    def get_timestamp(s): return s.ms
    def __bool__(s): return True
class FakePipe:
    def __init__(s): created["pipelines"] += 1; s.t = 1000.0; s.running = False
    def start(s, cfg):
        s.running = True
        intr = types.SimpleNamespace(fx=900.0, fy=900.0, ppx=640.0, ppy=360.0)
        return types.SimpleNamespace(get_stream=lambda st: types.SimpleNamespace(
            as_video_stream_profile=lambda: types.SimpleNamespace(get_intrinsics=lambda: intr)))
    def stop(s): s.running = False
    def wait_for_frames(s, timeout=5000):
        time.sleep(0.033); s.t += 33.0
        return FakeFrame(s.t)
rs = types.ModuleType("pyrealsense2")
rs.pipeline = FakePipe
rs.config = lambda: types.SimpleNamespace(enable_stream=lambda *a, **k: None)
rs.stream = types.SimpleNamespace(color=0); rs.format = types.SimpleNamespace(bgr8=0)
rs.rs2_fov = lambda intr: (70.3, 43.2)
sys.modules["pyrealsense2"] = rs

pm = types.ModuleType("pymavlink"); pm.mavutil = types.SimpleNamespace(mavlink=types.SimpleNamespace()); sys.modules["pymavlink"] = pm
sys.modules["pymavlink.mavutil"] = pm.mavutil

# ---------- 1. import must be side-effect free (the original bug) ----------
buf = io.StringIO(); old = sys.stdout; sys.stdout = buf
import vision_geotag
sys.stdout = old
assert created["pipelines"] == 0 and buf.getvalue() == "", "import ran code!"
assert hasattr(vision_geotag, "Geotagger")
print("1. import vision_geotag: no side effects, Geotagger exists  OK")

# ---------- 2. PixhawkController buffers through the real telemetry_loop ----------
import config
from pixhawk import PixhawkController
class Msg:
    def __init__(s, kind, **kw): s.kind = kind; s.__dict__.update(kw)
    def get_type(s): return s.kind
class FakeMaster:
    flightmode = "GUIDED"
    def __init__(s): s.queue = []
    def motors_armed(s): return False
    def recv_match(s, blocking=True, timeout=1, type=None):
        if s.queue: return s.queue.pop(0)
        time.sleep(0.01); return None
    def close(s): pass
ctl = PixhawkController(); ctl.master = FakeMaster(); ctl.connected = True
ev = threading.Event(); ev.set()
th = threading.Thread(target=ctl.telemetry_loop, args=(ev,), daemon=True); th.start()
ctl.master.queue += [
    Msg("GLOBAL_POSITION_INT", lat=185000000, lon=738000000, relative_alt=15000),
    Msg("ATTITUDE", roll=0.0, pitch=0.0, yaw=0.0),
    Msg("DISTANCE_SENSOR", current_distance=1500, min_distance=20, max_distance=4000),
    Msg("DISTANCE_SENSOR", current_distance=4500, min_distance=20, max_distance=4000),  # out of range
]
time.sleep(0.4)
now = time.time()
g = ctl.get_gps_at(now, 1.0); a = ctl.get_attitude_at(now, 1.0); l = ctl.get_lidar_at(now, 1.0)
assert g and abs(g["alt"] - 15.0) < 1e-9 and abs(g["lat"] - 18.5) < 1e-9, g
assert a and a["r"] == 0.0
assert l and abs(l["d"] - 15.0) < 1e-9, l               # cm -> m
assert len(ctl._lidar_buf) == 1, "out-of-range lidar must be dropped"
assert ctl.get_gps_at(now + 10, 0.5) is None             # stale sample rejected
assert ctl.get_lidar()["d"] == 15.0
assert ctl.get_gps()["relative_alt"] == 15.0
print("2. telemetry_loop buffers: cm->m, range filter, staleness, relative_alt  OK")
ev.clear()

# ---------- 3. Geotagger geometry through process_frame ----------
class Sensors:
    """Real PixhawkController API, filled with a hovering drone."""
    def __init__(s, lidar=True, yaw=0.0):
        s.c = PixhawkController(); s.lidar = lidar; s.yaw = yaw
    def fill(s, t):
        s.c._push(s.c._gps_buf, {"t": t, "lat": 18.5, "lon": 73.8, "alt": 15.0})
        s.c._push(s.c._att_buf, {"t": t, "r": 0.0, "p": 0.0, "y": s.yaw})
        if s.lidar: s.c._push(s.c._lidar_buf, {"t": t, "d": 15.0})
    def __getattr__(s, n): return getattr(s.c, n)

def run(box, lidar=True, frames=5):
    global BOXES
    BOXES[:] = [box]
    gt = vision_geotag.Geotagger("fake.pt", 0.5)
    sens = Sensors(lidar=lidar)
    out = []
    for _ in range(frames):
        t0 = time.time()
        for k in range(-5, 40): sens.fill(t0 + k * 0.05)
        vis, targets = gt.process_frame({}, {}, None, sensors=sens)
        out.append(targets)
    gt.shutdown()
    return out

def offset_m(lat, lon):
    n = (lat - 18.5) * math.pi / 180 * vision_geotag.EARTH_R
    e = (lon - 73.8) * math.pi / 180 * vision_geotag.EARTH_R * math.cos(math.radians(18.5))
    return e, n

os.chdir(WORK)
# pixel at image centre -> directly below the drone
res = run(FakeBox(630, 350, 650, 370, 1))
last = res[-1][0]; e, n = offset_m(last["lat"], last["lon"])
assert abs(e) < 0.05 and abs(n) < 0.05, (e, n)
assert [r[0]["n"] for r in res if r] == [1, 2, 3, 4, 5]
print(f"3a. centre pixel -> ({e:+.3f} E, {n:+.3f} N) m from drone, n grows 1..5  OK")
# 300 px right of centre at 15 m, fx=900 -> 5 m east
res = run(FakeBox(930, 350, 950, 370, 1)); last = res[-1][0]; e, n = offset_m(last["lat"], last["lon"])
assert abs(e - 5.0) < 0.05 and abs(n) < 0.05, (e, n)
print(f"3b. 300 px right  -> ({e:+.3f} E, {n:+.3f} N) m   expected (+5.0, 0)  OK")
# 300 px below centre -> 5 m south (image-down = behind the nose)
res = run(FakeBox(630, 650, 650, 670, 1)); last = res[-1][0]; e, n = offset_m(last["lat"], last["lon"])
assert abs(n + 5.0) < 0.05 and abs(e) < 0.05, (e, n)
print(f"3c. 300 px down   -> ({e:+.3f} E, {n:+.3f} N) m   expected (0, -5.0)  OK")
# no lidar sample: old code put EVERY target under the drone; must now still offset
res = run(FakeBox(930, 350, 950, 370, 1), lidar=False); last = res[-1][0]; e, n = offset_m(last["lat"], last["lon"])
assert abs(e - 5.0) < 0.05, (e, n)
print(f"3d. lidar missing -> ({e:+.3f} E, {n:+.3f} N) m   fallback plane works  OK")
# on the ground (no height) -> skipped instead of a garbage fix
class Ground(Sensors):
    def fill(s, t):
        s.c._push(s.c._gps_buf, {"t": t, "lat": 18.5, "lon": 73.8, "alt": 0.0})
        s.c._push(s.c._att_buf, {"t": t, "r": 0.0, "p": 0.0, "y": 0.0})
_S = Sensors; Sensors = Ground
res = run(FakeBox(930, 350, 950, 370, 1), lidar=False); Sensors = _S
assert all(r == [] for r in res)
print("3e. on the ground, no lidar -> no geotag produced  OK")

# ---------- 4. camera stop/start + shutdown ----------
BOXES[:] = []
gt = vision_geotag.Geotagger("fake.pt", 0.5)
gt.stop_camera(); assert gt.camera_running is False
t0 = time.time(); assert gt.process_frame(sensors=None) == (None, []); assert time.time() - t0 >= 0.4
gt.start_camera(); assert gt.camera_running and gt.rs_t0 is None
gt.shutdown(); gt.shutdown()
print("4. start/stop camera, idempotent shutdown  OK")

# ---------- 5. surveillance_loop end to end ----------
import main_surveillance
sent = []
class Radio:
    vision_thread = None
    def send_target(s, pid, lat, lon): sent.append((time.time(), pid, lat, lon)); return True
class FC(Sensors):
    connected = True
    def __init__(s):
        super().__init__()
        s.c.telemetry.update(lat=18.5, lon=73.8, alt=15.0, roll=0.0, pitch=0.0, yaw=0.0)
    def get_gps(s): return s.c.get_gps()
    def get_attitude(s): return s.c.get_attitude()
    def get_lidar(s): return s.c.get_lidar()
fc = FC(); radio = Radio()
BOXES[:] = [FakeBox(930, 350, 950, 370, 7)]
run_ev = threading.Event(); run_ev.set()
feeder_stop = threading.Event()
def feeder():
    while not feeder_stop.is_set():
        fc.fill(time.time()); time.sleep(0.02)
threading.Thread(target=feeder, daemon=True).start()
lt = threading.Thread(target=main_surveillance.surveillance_loop, args=(fc, run_ev, radio), daemon=True); lt.start()
time.sleep(3.2)
run_ev.clear(); feeder_stop.set(); lt.join(5)
assert radio.vision_thread is not None, "vision handle not registered on radio"
assert len(sent) >= 2, sent
gaps = [b[0] - a[0] for a, b in zip(sent, sent[1:])]
assert min(gaps) >= 0.95, gaps                       # <= 1 message/s per person
e, n = offset_m(sent[-1][2], sent[-1][3]); assert abs(e - 5.0) < 0.1
print(f"5. surveillance_loop: {len(sent)} TARGET msgs in 3 s, min gap {min(gaps):.2f}s, last at ({e:+.2f} E, {n:+.2f} N)  OK")
print("ALL JETSON WIRING TESTS PASSED")
