import time
import threading
import traceback
from vision_geotag import Geotagger, MIN_POINTS_FILTER

import sys
import os

# Force Python to read config and modules ONLY from the jetson subfolder
BASE_DIR = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(BASE_DIR, "jetson"))

TARGET_SEND_PERIOD_S = 1.0       # max rate of TARGET messages per person
TARGET_REBROADCAST_S = 10.0      # re-send every confirmed target now and then (radio drops packets)


def surveillance_loop(pixhawk_controller, running_event, radio_link):
    """
    Runs continuously in the background on the Jetson.
    Requires the active PixhawkController instance to prevent serial port conflicts.
    """
    vision = None
    try:
        print("[INFO] Initializing Vision Pipeline (CUDA/RealSense)...")
        vision = Geotagger(model_path="/home/jetson-arsc/radio_gcs/models/bestest.pt", conf_threshold=0.50)

        # Lets the radio bridge reach start/stop camera and model-swap commands.
        radio_link.vision_thread = vision

        # Wait for FC code to establish the connection
        while not pixhawk_controller.connected and running_event.is_set():
            print("[INFO] Vision waiting for Pixhawk telemetry...")
            time.sleep(1)

        print("[INFO] Flight Controller linked. Starting main vision loop.")

        last_sent = {}          # pid -> time of last TARGET message
        latest = {}             # pid -> (lat, lon) of every confirmed target
        last_rebroadcast = time.time()

        while running_event.is_set():
            try:
                gps_data = pixhawk_controller.get_gps()
                att_data = pixhawk_controller.get_attitude()
                lidar_data = pixhawk_controller.get_lidar()

                # Ensure we have a valid GPS lock before doing math
                if gps_data["lat"] is None:
                    time.sleep(0.2)
                    continue

                # Grab a frame; GPS/attitude/lidar are looked up at the frame's timestamp.
                vis_frame, targets = vision.process_frame(
                    gps_data, att_data, lidar_data, sensors=pixhawk_controller)

                now = time.time()

                # Broadcast resulting targets down to the Asus Laptop GCS
                for target in targets:
                    if target["n"] < MIN_POINTS_FILTER:
                        continue        # not enough points yet - skip the noisy raw fix
                    latest[target["pid"]] = (target["lat"], target["lon"])
                    if now - last_sent.get(target["pid"], 0.0) < TARGET_SEND_PERIOD_S:
                        continue
                    if radio_link.send_target(target["pid"], target["lat"], target["lon"]):
                        last_sent[target["pid"]] = now
                        print(f"[BROADCAST] TARGET:{target['pid']},{target['lat']:.6f},{target['lon']:.6f}")

                if now - last_rebroadcast >= TARGET_REBROADCAST_S:
                    for pid, (lat, lon) in latest.items():
                        radio_link.send_target(pid, lat, lon)
                    last_rebroadcast = now

            except Exception as e:
                print(f"[ERROR] Vision iteration failed: {e}")
                time.sleep(1)

    except Exception:
        print("[ERROR] Surveillance loop stopped:")
        traceback.print_exc()
    finally:
        if vision is not None:
            vision.shutdown()
        print("[INFO] Vision pipeline safely closed.")

# This allows testing the script standalone if needed
if __name__ == "__main__":
    from pixhawk import PixhawkController
    
    # Dummy radio class for testing
    class DummyRadio:
        def send_target(self, pid, lat, lon):
            return True
        
    dummy_event = threading.Event()
    dummy_event.set()
    
    fc = PixhawkController()
    threading.Thread(target=fc.telemetry_loop, args=(dummy_event,), daemon=True).start()
    
    surveillance_loop(fc, dummy_event, DummyRadio())
