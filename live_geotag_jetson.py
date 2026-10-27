#!/usr/bin/env python3

import os, sys, time, threading
from datetime import datetime
from math import cos, radians, degrees
from collections import defaultdict
import torch
import cv2
import pandas as pd
import numpy as np
from pymavlink import mavutil
from ultralytics import YOLO

# ================= USER CONFIG =================
MODEL_PATH = r"C:\Users\Asus\OneDrive\Desktop\Dhanvi\ARSC\Nidar\bestest.pt"
OUTPUT_FOLDER = r"C:\Users\Asus\OneDrive\Desktop\Dhanvi\ARSC\Nidar"
CSV_OUT_BASE = "human_detections_geotagged"
CONF_THRESHOLD = 0.50  # Enforced to prevent phantom waypoints

RS_COLOR_W, RS_COLOR_H, RS_FPS = 1280, 720, 30
PIXHAWK_PORT = "/dev/ttyACM0"
PIXHAWK_BAUD = 115200

LIDAR_ENABLED = True
GROUND_ALT_AMSL = 0.0
EARTH_R = 6378137.0

# --- TIME-BASED FILTERING PARAMS ---
TIME_WINDOW_SEC = 2.0
MIN_POINTS_FILTER = 3
PERSON_MERGE_DIST_M = 5.0

os.makedirs(OUTPUT_FOLDER, exist_ok=True)

# ================= TIME SYNC =================
rs_t0 = None      # RealSense time (seconds)
sys_t0 = None     # Corresponding system time

# ================= LOAD YOLO =================
print("[INFO] Loading YOLO model...")
model = YOLO(MODEL_PATH)

if torch.cuda.is_available():
    model.to("cuda:0")
    print("[INFO] YOLO moved to CUDA (Jetson GPU)")
else:
    print("[WARN] CUDA not available — running on CPU")

device = model.device if hasattr(model, "device") else "unknown"
print(f"[INFO] YOLO model loaded successfully on {device}")

# ================= DATAFRAME =================
columns = [
    "Serial_No","Track_ID","Person_ID","Frame_Number","Frame_Timestamp_s",
    "Center_X","Center_Y",
    "Geo_Lat_Raw","Geo_Lon_Raw",
    "Geo_Lat_Filtered","Geo_Lon_Filtered"
]
df = pd.DataFrame(columns=columns)

# ================= GEOMETRY =================
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

# ================= FILTERING =================
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

# ================= ID MANAGEMENT =================
track_to_person = {}
person_buffers = defaultdict(list)
next_person_id = 1

def prune_buffer(buf, now):
    return [p for p in buf if now - p["t"] <= TIME_WINDOW_SEC]

def assign_person(track_id, lat, lon):
    global next_person_id
    if track_id in track_to_person:
        return track_to_person[track_id]

    for pid,buf in person_buffers.items():
        if len(buf) < MIN_POINTS_FILTER: 
            continue
        mlat,mlon = median_latlon(buf)
        e,n = latlon_to_enu(mlat,mlon,lat,lon)
        if np.hypot(e,n) < PERSON_MERGE_DIST_M:
            track_to_person[track_id] = pid
            return pid

    pid = next_person_id
    next_person_id += 1
    track_to_person[track_id] = pid
    return pid

# ================= MAVLINK =================
gps_buf, att_buf, lidar_buf = [],[],[]

def get_nearest(buf,t,dt=0.5):
    if not buf: return None
    s = min(buf,key=lambda x:abs(x["t"]-t))
    return s if abs(s["t"]-t)<=dt else None

def mav_reader():
    while True:
        m = mav.recv_match(blocking=True,timeout=1)
        if not m: continue
        t=time.time()
        if m.get_type()=="GLOBAL_POSITION_INT":
            gps_buf.append({"t":t,"lat":m.lat/1e7,"lon":m.lon/1e7,"alt":m.alt/1000})
        elif m.get_type()=="ATTITUDE":
            att_buf.append({"t":t,"r":m.roll,"p":m.pitch,"y":m.yaw})
        elif m.get_type()=="DISTANCE_SENSOR":
            lidar_buf.append({"t":t,"d":m.current_distance/100})

# ================= START SENSORS =================
import pyrealsense2 as rs
pipe=rs.pipeline()
cfg=rs.config()
cfg.enable_stream(rs.stream.color,RS_COLOR_W,RS_COLOR_H,rs.format.bgr8,RS_FPS)
profile=pipe.start(cfg)
intr=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
K=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]])
hfov, vfov = rs.rs2_fov(intr)
print(f"[INFO] Camera FOV - Horizontal: {hfov:.1f}°, Vertical: {vfov:.1f}°")

print("[INFO] Waiting for Pixhawk heartbeat...")
mav=mavutil.mavlink_connection(PIXHAWK_PORT,baud=PIXHAWK_BAUD)
mav.wait_heartbeat()
print("[INFO] Pixhawk connected (heartbeat received)")

threading.Thread(target=mav_reader,daemon=True).start()

# ================= MAIN LOOP =================
serial=1; frame=0
try:
    while True:
        f=pipe.wait_for_frames().get_color_frame()
        img=np.asanyarray(f.get_data())
        vis = img.copy()     
        
        rs_ts = f.get_timestamp() * 1e-3  

        if rs_t0 is None:
            rs_t0 = rs_ts
            sys_t0 = time.time()
            print("[INFO] Time sync initialized (RealSense → system)")

        ts = sys_t0 + (rs_ts - rs_t0)

        with torch.no_grad():
            start_infer = time.time()
            res = model.track(
                img,
                persist=True,
                verbose=False,
                half=True,      
                imgsz=640,
                conf=CONF_THRESHOLD  # <-- Added strictly to 0.50
            )[0]
            end_infer = time.time()
            yolo_time = end_infer - start_infer

        if not res.boxes:
            frame+=1
            continue

        detection_happened = False

        for box in res.boxes:
            if int(box.cls[0])!=0: continue

            detection_happened = True

            x1,y1,x2,y2 = map(int,box.xyxy[0])
            cx,cy = (x1+x2)/2,(y1+y2)/2
            tid = int(box.id[0]) if box.id is not None else -1

            gps=get_nearest(gps_buf,ts,5)
            att=get_nearest(att_buf,ts,5)
            if not gps or not att: continue

            ground = gps["alt"]
            if LIDAR_ENABLED:
                l=get_nearest(lidar_buf,ts)
                if l: ground -= l["d"]

            ray = pixel_to_camera_ray(cx,cy,K)
            ray_ned = rpy_to_R(att["r"],att["p"],att["y"]) @ ray
            hit = intersect(gps["lat"],gps["lon"],gps["alt"],ray_ned,ground)
            if not hit: continue
            lat,lon = hit

            pid = assign_person(tid,lat,lon)
            person_buffers[pid].append({"lat":lat,"lon":lon,"t":ts})
            person_buffers[pid] = prune_buffer(person_buffers[pid],ts)

            filt_lat,filt_lon = lat,lon
            if len(person_buffers[pid])>=MIN_POINTS_FILTER:
                filt_lat,filt_lon = median_latlon(person_buffers[pid])
                r=ransac_latlon(person_buffers[pid])
                if r: filt_lat,filt_lon = r

            # ===== TRANSMIT VIA PURE TELEMETRY =====
            msg = f"TARGET: {filt_lat:.5f}, {filt_lon:.5f}"
            mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, msg.encode())

            # ===== DRAW BOX & TEXT =====
            cv2.rectangle(vis,(x1,y1),(x2,y2),(0,255,0),2)
            label=f"PID:{pid} TID:{tid}"
            cv2.putText(vis,label,(x1,y1-8),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

            df.loc[len(df)] = [
                serial,tid,pid,frame,round(ts,3),
                round(cx,1),round(cy,1),
                lat,lon,filt_lat,filt_lon
            ]
            serial+=1

        if detection_happened:
            fname = f"{OUTPUT_FOLDER}/frame_{frame:06d}_{int(ts*1000)}.jpg"
            cv2.imwrite(fname,vis)

        frame+=1

except KeyboardInterrupt:
    pass
finally:
    pipe.stop()
    
    csv_name = f"{CSV_OUT_BASE}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    df.to_csv(csv_name, index=False)
   
    print("[INFO] Computing final location per Track_ID...")
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


    