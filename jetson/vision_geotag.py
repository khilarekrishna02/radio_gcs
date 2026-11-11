#!/usr/bin/env python3
"""Vision + geotagging for Drone 1.

The geometry and filtering functions in this file are unchanged from the
original script. What changed:
  * nothing runs at import time any more - the sensor/model/loop code that used
    to sit at module level now lives in the Geotagger class
  * GPS / attitude / lidar come from PixhawkController (one owner of the
    Pixhawk port) and are looked up at the FRAME timestamp
  * detections are returned to the caller instead of being pushed into the
    Pixhawk as STATUSTEXT
"""

import os, time, threading
from datetime import datetime
from math import cos, radians, degrees
from collections import defaultdict
import torch
import cv2
import pandas as pd
import numpy as np
from ultralytics import YOLO

# ================= USER CONFIG =================
MODEL_PATH = r"bestest.pt"
OUTPUT_FOLDER = os.getenv("VISION_OUTPUT_DIR", os.path.expanduser("~/nidar_out"))
CSV_OUT_BASE = "human_detections_geotagged"
CONF_THRESHOLD = 0.50  # Enforced to prevent phantom waypoints

RS_COLOR_W, RS_COLOR_H, RS_FPS = 1280, 720, 30

LIDAR_ENABLED = True
# PixhawkController reports altitude RELATIVE TO HOME, so the ground-plane
# fallback (used only when there is no fresh lidar sample) is 0.0 in that frame.
GROUND_ALT_AMSL = 0.0
EARTH_R = 6378137.0
R_cam_to_body = np.array([
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
    [0.0,  0.0, 1.0],
])

# --- TIME-BASED FILTERING PARAMS ---
TIME_WINDOW_SEC = 2.0
MIN_POINTS_FILTER = 3
PERSON_MERGE_DIST_M = 5.0

# --- SENSOR MATCHING (max age of the sample matched to a frame timestamp) ---
GPS_MAX_AGE_S = 0.5
ATT_MAX_AGE_S = 0.15
LIDAR_MAX_AGE_S = 0.5
MIN_HEIGHT_ABOVE_GROUND_M = 0.5   # below this the ray/ground intersection is meaningless

SAVE_DETECTION_FRAMES = True      # write an annotated JPEG for every frame with a detection

# ================= DATAFRAME COLUMNS =================
columns = [
    "Serial_No","Track_ID","Person_ID","Frame_Number","Frame_Timestamp_s",
    "Center_X","Center_Y",
    "Geo_Lat_Raw","Geo_Lon_Raw",
    "Geo_Lat_Filtered","Geo_Lon_Filtered"
]

# ================= GEOMETRY (unchanged) =================
def pixel_to_camera_ray(u,v,K):
    x = (u-K[0,2])/K[0,0]
    y = (v-K[1,2])/K[1,1]
    r = np.array([x,y,1.0])
    return r/np.linalg.norm(r)

def rpy_to_R(roll,pitch,yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])

def ned_to_enu(v): 
    return np.array([v[1],v[0],-v[2]])

def enu_to_latlon(lat0,lon0,e,n):
    return (
        lat0 + degrees(n/EARTH_R),
        lon0 + degrees(e/(EARTH_R*cos(radians(lat0))))
    )

def intersect(lat,lon,alt,ray_ned,ground):
    e,n,u = ned_to_enu(ray_ned)
    if abs(u) < 1e-6: 
        return None
    t = (ground-alt)/u
    if t < 0:
        return None
    return enu_to_latlon(lat,lon,e*t,n*t)

# ================= FILTERING (unchanged) =================
def latlon_to_enu(lat0,lon0,lat,lon):
    dlat = radians(lat-lat0)
    dlon = radians(lon-lon0)
    return (
        dlon*EARTH_R*cos(radians(lat0)),
        dlat*EARTH_R
    )

def median_latlon(pts):
    return float(np.median([p["lat"] for p in pts])), \
           float(np.median([p["lon"] for p in pts]))

def ransac_latlon(pts,thresh=3.0):
    if len(pts) < 3: 
        return None
    lat0,lon0 = pts[0]["lat"], pts[0]["lon"]
    enu = np.array([latlon_to_enu(lat0,lon0,p["lat"],p["lon"]) for p in pts])
    best=[]
    for i in range(len(enu)):
        d = np.linalg.norm(enu-enu[i],axis=1)
        inl = np.where(d<thresh)[0]
        if len(inl) > len(best): 
            best = inl
    if len(best) < 3: 
        return None
    return median_latlon([pts[i] for i in best])

def prune_buffer(buf, now):
    return [p for p in buf if now - p["t"] <= TIME_WINDOW_SEC]


# ================= GEOTAGGER =================
class Geotagger:
    """Owns the RealSense camera and the YOLO model. No Pixhawk connection of its own."""

    def __init__(self, model_path=MODEL_PATH, conf_threshold=CONF_THRESHOLD):
        import pyrealsense2 as rs
        os.makedirs(OUTPUT_FOLDER, exist_ok=True)
        self.conf_threshold = conf_threshold
        self._cam_lock = threading.Lock()
        self._model_lock = threading.Lock()
        self._closed = False

        # ----- time sync -----
        self.rs_t0 = None      # RealSense time (seconds)
        self.sys_t0 = None     # Corresponding system time

        # ----- YOLO -----
        print("[INFO] Loading YOLO model...")
        self.model = YOLO(model_path)
        self.model_name = os.path.basename(str(model_path))
        self._place_model()

        # ----- camera -----
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, RS_COLOR_W, RS_COLOR_H, rs.format.bgr8, RS_FPS)
        profile = self.pipe.start(self.cfg)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
        hfov, vfov = rs.rs2_fov(intr)
        print(f"[INFO] Camera FOV - Horizontal: {hfov:.1f}\u00b0, Vertical: {vfov:.1f}\u00b0")
        self.camera_running = True

        # ----- ID management -----
        self.track_to_person = {}
        self.person_buffers = defaultdict(list)
        self.next_person_id = 1
        self.last_known_loc = {}

        # ----- logging -----
        self.rows = []
        self.serial = 1
        self.frame = 0

    # ---------- model ----------
    def _place_model(self):
        self.use_half = torch.cuda.is_available()
        if torch.cuda.is_available():
            self.model.to("cuda:0")
            print("[INFO] YOLO moved to CUDA (Jetson GPU)")
        else:
            print("[WARN] CUDA not available \u2014 running on CPU")
        device = self.model.device if hasattr(self.model, "device") else "unknown"
        print(f"[INFO] YOLO model loaded successfully on {device}")

    def load_model(self, new_model):
        print(f"[INFO] Swapping YOLO model to {new_model}...")
        new = YOLO(new_model)
        with self._model_lock:
            self.model = new
            self.model_name = os.path.basename(str(new_model))
            self._place_model()

    # ---------- camera control (called from the radio thread) ----------
    def stop_camera(self):
        with self._cam_lock:
            if self.camera_running:
                print("[INFO] Pausing RealSense pipeline...")
                self.pipe.stop()
                self.camera_running = False

    def start_camera(self):
        with self._cam_lock:
            if not self.camera_running:
                print("[INFO] Resuming RealSense pipeline...")
                self.pipe.start(self.cfg)
                self.camera_running = True
                self.rs_t0 = None   # re-sync RealSense -> system time after a restart

    # ---------- person IDs (logic unchanged) ----------
    def assign_person(self, track_id, lat, lon):
        if track_id in self.track_to_person:
            return self.track_to_person[track_id]

        # Check against permanently saved locations instead of the 2-second buffer
        for pid, loc in self.last_known_loc.items():
            e, n = latlon_to_enu(loc["lat"], loc["lon"], lat, lon)
            if np.hypot(e, n) < PERSON_MERGE_DIST_M:
                self.track_to_person[track_id] = pid
                self.last_known_loc[pid] = {"lat": lat, "lon": lon} # Update location
                return pid

        # Create brand new ID
        pid = self.next_person_id
        self.next_person_id += 1
        self.track_to_person[track_id] = pid
        self.last_known_loc[pid] = {"lat": lat, "lon": lon}
        return pid

    # ---------- sensor lookup at the frame timestamp ----------
    def _sensor_snapshot(self, ts, gps_data, att_data, lidar_data, sensors):
        if sensors is not None:
            gps = sensors.get_gps_at(ts, GPS_MAX_AGE_S)
            att = sensors.get_attitude_at(ts, ATT_MAX_AGE_S)
            l = sensors.get_lidar_at(ts, LIDAR_MAX_AGE_S) if LIDAR_ENABLED else None
            return gps, att, l
        # Fallback (bench tests): use the latest values handed in by the caller
        gps = att = l = None
        if gps_data and gps_data.get("lat") is not None and gps_data.get("alt") is not None:
            gps = {"t": ts, "lat": gps_data["lat"], "lon": gps_data["lon"], "alt": gps_data["alt"]}
        if att_data and att_data.get("roll") is not None:
            att = {"t": ts, "r": att_data["roll"], "p": att_data["pitch"], "y": att_data["yaw"]}
        if LIDAR_ENABLED and isinstance(lidar_data, dict) and lidar_data.get("d") is not None:
            l = {"t": ts, "d": lidar_data["d"]}
        return gps, att, l

    # ---------- main per-frame work ----------
    def process_frame(self, gps_data=None, att_data=None, lidar_data=None, sensors=None):
        """Grab one frame, detect, geotag. Returns (annotated_frame, targets).

        targets: list of dicts {pid, tid, lat, lon, n} where lat/lon is the
        filtered location and n is how many points back it up.
        """
        if not self.camera_running:
            time.sleep(0.5)
            return None, []

        with self._cam_lock:
            if not self.camera_running:
                return None, []
            try:
                frames = self.pipe.wait_for_frames(2000)
            except RuntimeError as exc:
                print(f"[WARN] RealSense frame timeout: {exc}")
                return None, []
        f = frames.get_color_frame()
        if not f:
            return None, []
        img = np.asanyarray(f.get_data())
        vis = img.copy()

        rs_ts = f.get_timestamp() * 1e-3

        if self.rs_t0 is None:
            self.rs_t0 = rs_ts
            self.sys_t0 = time.time()
            print("[INFO] Time sync initialized (RealSense \u2192 system)")

        ts = self.sys_t0 + (rs_ts - self.rs_t0)

        with self._model_lock:
            with torch.no_grad():
                start_infer = time.time()
                res = self.model.track(
                    img,
                    persist=True,
                    verbose=False,
                    half=self.use_half,
                    imgsz=640,
                    conf=self.conf_threshold
                )[0]
                end_infer = time.time()
                yolo_time = end_infer - start_infer

        if not res.boxes:
            self.frame += 1
            return vis, []

        gps, att, l = self._sensor_snapshot(ts, gps_data, att_data, lidar_data, sensors)

        targets = []
        detection_happened = False

        for box in res.boxes:
            if int(box.cls[0]) != 0: continue

            detection_happened = True

            x1,y1,x2,y2 = map(int,box.xyxy[0])
            cx,cy = (x1+x2)/2,(y1+y2)/2
            tid = int(box.id[0]) if box.id is not None else -1

            if not gps or not att: continue

            # Ground plane: lidar when we have a fresh sample, otherwise the
            # fallback plane (home height). Without this fallback a missing lidar
            # sample made t = 0 and every target landed under the drone.
            ground = GROUND_ALT_AMSL
            if LIDAR_ENABLED and l:
                ground = gps["alt"] - l["d"] * cos(att["r"]) * cos(att["p"])
            if gps["alt"] - ground < MIN_HEIGHT_ABOVE_GROUND_M:
                continue

            ray = pixel_to_camera_ray(cx,cy,self.K)
            ray_body = R_cam_to_body @ ray
            ray_ned = rpy_to_R(att["r"], att["p"], att["y"]) @ ray_body
            hit = intersect(gps["lat"],gps["lon"],gps["alt"],ray_ned,ground)
            if not hit: continue
            lat,lon = hit

            pid = self.assign_person(tid,lat,lon)
            self.person_buffers[pid].append({"lat":lat,"lon":lon,"t":ts})
            self.person_buffers[pid] = prune_buffer(self.person_buffers[pid],ts)

            filt_lat,filt_lon = lat,lon
            if len(self.person_buffers[pid])>=MIN_POINTS_FILTER:
                filt_lat,filt_lon = median_latlon(self.person_buffers[pid])
                r=ransac_latlon(self.person_buffers[pid])
                if r: filt_lat,filt_lon = r

            targets.append({"pid": pid, "tid": tid, "lat": filt_lat, "lon": filt_lon,
                            "n": len(self.person_buffers[pid])})

            # ===== DRAW BOX & TEXT =====
            cv2.rectangle(vis,(x1,y1),(x2,y2),(0,255,0),2)
            label=f"PID:{pid} TID:{tid}"
            cv2.putText(vis,label,(x1,y1-8),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

            self.rows.append([
                self.serial,tid,pid,self.frame,round(ts,3),
                round(cx,1),round(cy,1),
                lat,lon,filt_lat,filt_lon
            ])
            self.serial+=1

        if detection_happened and SAVE_DETECTION_FRAMES:
            fname = f"{OUTPUT_FOLDER}/frame_{self.frame:06d}_{int(ts*1000)}.jpg"
            cv2.imwrite(fname,vis)

        self.frame+=1
        return vis, targets

    # ---------- shutdown ----------
    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        try:
            with self._cam_lock:
                if self.camera_running:
                    self.pipe.stop()
                    self.camera_running = False
        except Exception as exc:
            print(f"[WARN] Camera stop failed: {exc}")

        df = pd.DataFrame(self.rows, columns=columns)
        csv_name = f"{CSV_OUT_BASE}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        df.to_csv(csv_name, index=False)

        print("[INFO] Computing final location per Track_ID...")
        try:
            valid = df.dropna(subset=["Geo_Lat_Filtered", "Geo_Lon_Filtered"])
            final_track_locations = (
                valid.groupby("Track_ID")[["Geo_Lat_Filtered", "Geo_Lon_Filtered"]]
                     .median()
                     .reset_index()
                     .rename(columns={
                         "Geo_Lat_Filtered": "Final_Lat",
                         "Geo_Lon_Filtered": "Final_Lon"
                     })
            )
            final_track_locations.to_csv("final_locations_per_track.csv", index=False)
            print("[INFO] Final Track locations saved to final_locations_per_track.csv")
        except Exception as exc:
            print(f"[WARN] Could not compute final track locations: {exc}")
