#!/usr/bin/env python3
"""Pre-flight check for the Jetson. Run it from the jetson folder:

    python3 jetson_selfcheck.py                       # software, ports, files
    python3 jetson_selfcheck.py --camera              # + grab frames from the RealSense and measure FPS
    python3 jetson_selfcheck.py --model bestest.pt    # + run the model and measure inference FPS

It only reads and reports; it never changes anything. Exit code 0 = no failures.
"""


import argparse
import warnings
import glob
import importlib
import io
import os
import shutil
import subprocess
import sys
import time

import sys
import os

# Force Python to look inside your specific subfolders for imports
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(BASE_DIR, "laptop"))
sys.path.append(os.path.join(BASE_DIR, "laptop"))

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))


warnings.simplefilter("ignore")
fails = warns = 0


def ok(msg):
    print(f"[ OK ] {msg}")


def warn(msg):
    global warns
    warns += 1
    print(f"[WARN] {msg}")


def bad(msg):
    global fails
    fails += 1
    print(f"[FAIL] {msg}")


def section(title):
    print(f"\n=== {title} ===")


def check_python_packages():
    section("Python packages")
    print(f"python {sys.version.split()[0]} ({sys.executable})")
    for mod, why in [("numpy", ""), ("pandas", ""), ("cv2", "OpenCV"), ("torch", "PyTorch"),
                     ("ultralytics", "YOLO"), ("pyrealsense2", "RealSense camera"),
                     ("pymavlink", "Pixhawk link"), ("serial", "pyserial - radio"), ("flask", "")]:
        try:
            m = importlib.import_module(mod)
            ok(f"{mod:14s} {getattr(m, '__version__', '')}")
        except Exception as exc:
            bad(f"{mod:14s} cannot import ({why}): {exc}")


def check_gpu():
    section("GPU")
    try:
        import torch
        if torch.cuda.is_available():
            ok(f"CUDA available: {torch.cuda.get_device_name(0)} (torch {torch.__version__})")
        else:
            bad(f"torch {torch.__version__} has NO CUDA - YOLO would run on the CPU. Install NVIDIA's PyTorch build for your JetPack.")
    except Exception as exc:
        bad(f"torch not usable: {exc}")


def check_ports_and_config():
    section("Config and serial ports")
    try:
        import config
    except Exception as exc:
        bad(f"cannot import config.py: {exc}")
        return
    for name in ("RADIO_PORT", "RADIO_BAUD", "PIXHAWK_PORT", "PIXHAWK_BAUD", "TELEMETRY_PERIOD", "MISSION_DIR", "LIDAR_ENABLED"):
        if hasattr(config, name):
            ok(f"config.{name} = {getattr(config, name)}")
        else:
            bad(f"config.{name} is missing")
    for label in ("RADIO_PORT", "PIXHAWK_PORT"):
        path = getattr(config, label, None)
        if not path:
            continue
        if not os.path.exists(path):
            bad(f"{label} {path} does not exist (is it plugged in? see the by-id list below)")
        elif not os.access(path, os.R_OK | os.W_OK):
            bad(f"{label} {path} exists but you cannot read/write it - add your user to the 'dialout' group, then log out and in")
        else:
            ok(f"{label} {path} exists and is accessible")
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    if by_id:
        print("stable names you can use in the service file:")
        for p in by_id:
            print(f"   {p} -> {os.path.realpath(p)}")
    else:
        warn("no /dev/serial/by-id entries (radio / Pixhawk not plugged in?)")


def check_files():
    section("Files")
    
    for name in ("models/bestest.pt", "models/yolov8n.pt", "models/yolov8s.pt"):
        if os.path.isfile(name):
            ok(f"{name} present ({os.path.getsize(name) / 1e6:.1f} MB)")
        else:
            (bad if name == "bestest.pt" else warn)(f"{name} not found in {os.getcwd()} (the page's model list needs it here; the Jetson cannot download it in the field)")
    crlf = []
    for pattern in ("*.py", "*.service"):
        for path in glob.glob(pattern):
            with open(path, "rb") as f:
                if b"\r\n" in f.read():
                    crlf.append(path)
    if crlf:
        bad("Windows line endings (CRLF) in: " + ", ".join(crlf) + "  -> fix: sed -i 's/\\r$//' <file>  (breaks .service files)")
    else:
        ok("no CRLF line endings")
    free = shutil.disk_usage(os.path.expanduser("~")).free / 1e9
    (ok if free > 2 else warn)(f"{free:.1f} GB free in home (detection frames are saved to ~/nidar_out)")


def check_import_is_clean():
    section("vision_geotag import")
    buf, old = io.StringIO(), sys.stdout
    t0 = time.time()
    try:
        sys.stdout = buf
        import vision_geotag
    except Exception as exc:
        sys.stdout = old
        bad(f"importing vision_geotag failed: {exc}")
        return
    finally:
        sys.stdout = old
    if not hasattr(vision_geotag, "Geotagger"):
        bad("vision_geotag has no Geotagger class (old file?)")
    elif buf.getvalue().strip():
        bad("importing vision_geotag printed output / ran code - it must be silent:\n" + buf.getvalue()[:300])
    else:
        ok(f"import is silent and Geotagger exists ({time.time() - t0:.1f}s, mostly torch/ultralytics loading)")


def check_protocol():
    section("protocol.py")
    try:
        import protocol
        if hasattr(protocol, "parse_qgc_wpl") and hasattr(protocol, "encode_message"):
            ok(f"protocol imported from {protocol.__file__}")
        else:
            bad(f"{protocol.__file__} is missing parse_qgc_wpl/encode_message - use the full protocol.py")
    except Exception as exc:
        bad(f"cannot import protocol ({exc}). protocol.py must sit in this folder, next to flask_app.py")


def check_service():
    section("systemd service")
    for cmd in (["systemctl", "is-enabled", "drone-gcs"], ["systemctl", "is-active", "drone-gcs"]):
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout.strip()
            print(f"   {' '.join(cmd[1:])}: {out or 'unknown'}")
        except Exception:
            print("   systemctl not available")
            return
    print("   (not enabled yet is fine until the manual tests pass)")


def check_camera():
    section("RealSense camera")
    try:
        import pyrealsense2 as rs
        devices = rs.context().devices
        if len(devices) == 0:
            bad("no RealSense device found (cable, USB 3 port, or another program is using it)")
            return
        dev = devices[0]
        usb = dev.get_info(rs.camera_info.usb_type_descriptor) if dev.supports(rs.camera_info.usb_type_descriptor) else "?"
        ok(f"{dev.get_info(rs.camera_info.name)} on USB {usb}")
        if str(usb).startswith("2"):
            warn("camera is on USB 2.x - use a USB 3 port/cable for 1280x720 at 30 FPS")
        pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        pipe.start(cfg)
        try:
            for _ in range(5):
                pipe.wait_for_frames(5000)
            t0, n = time.time(), 60
            for _ in range(n):
                frames = pipe.wait_for_frames(5000)
            fps = n / (time.time() - t0)
            shape = frames.get_color_frame().get_data()
            (ok if fps > 25 else warn)(f"camera delivers {fps:.1f} FPS at 1280x720")
        finally:
            pipe.stop()
    except Exception as exc:
        bad(f"camera test failed: {exc}")


def check_model(path):
    section(f"Model speed: {path}")
    try:
        import numpy as np
        import torch
        from ultralytics import YOLO
        model = YOLO(path)
        if torch.cuda.is_available():
            model.to("cuda:0")
        frame = np.zeros((720, 1280, 3), np.uint8)
        half = torch.cuda.is_available()
        for _ in range(3):
            model.track(frame, persist=True, verbose=False, half=half, imgsz=640, conf=0.5)
        n, t0 = 20, time.time()
        for _ in range(n):
            model.track(frame, persist=True, verbose=False, half=half, imgsz=640, conf=0.5)
        fps = n / (time.time() - t0)
        msg = f"{path}: {fps:.1f} inferences/s on {'GPU' if half else 'CPU'}"
        (ok if fps >= 8 else warn)(msg + ("" if fps >= 8 else "  (low - fly slower/higher or use a smaller model)"))
    except Exception as exc:
        bad(f"model test failed: {exc}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", action="store_true", help="also grab frames from the RealSense")
    ap.add_argument("--model", metavar="FILE.pt", help="also time inference with this model")
    args = ap.parse_args()
    sys.path.insert(0, os.getcwd())
    check_python_packages()
    check_gpu()
    check_ports_and_config()
    check_files()
    check_import_is_clean()
    check_protocol()
    check_service()
    if args.camera:
        check_camera()
    if args.model:
        check_model(args.model)
    print(f"\nSummary: {fails} failure(s), {warns} warning(s)")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
