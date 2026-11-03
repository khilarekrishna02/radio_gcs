import time
import threading
from vision_geotag import Geotagger

def surveillance_loop(pixhawk_controller, running_event, radio_link):
    """
    Runs continuously in the background on the Jetson.
    Requires the active PixhawkController instance to prevent serial port conflicts.
    """
    print("[INFO] Initializing Vision Pipeline (CUDA/RealSense)...")
    vision = Geotagger(model_path="bestest.pt", conf_threshold=0.50)
    
    # Wait for FC code to establish the connection
    while not pixhawk_controller.connected and running_event.is_set():
        print("[INFO] Vision waiting for Pixhawk telemetry...")
        time.sleep(1)

    print("[INFO] Flight Controller linked. Starting main vision loop.")

    try:
        while running_event.is_set():
            # Pull the cached dictionaries we just added to pixhawk.py
            gps_data = pixhawk_controller.get_gps()
            att_data = pixhawk_controller.get_attitude()
            lidar_data = pixhawk_controller.get_lidar()

            # Ensure we have a valid GPS lock before doing math
            if gps_data["lat"] is None:
                continue

            # Feed telemetry and grab a frame to process
            vis_frame, targets = vision.process_frame(gps_data, att_data, lidar_data)

            # Broadcast resulting targets down to the Asus Laptop GCS
            for target in targets:
                msg = f"TARGET:{target['pid']},{target['lat']:.6f},{target['lon']:.6f}"
                radio_link.send_message(msg) 
                print(f"[BROADCAST] {msg}")

    except Exception as e:
        print(f"[ERROR] Surveillance loop stopped: {e}")
    finally:
        vision.shutdown()
        print("[INFO] Vision pipeline safely closed.")

# This allows testing the script standalone if needed
if __name__ == "__main__":
    from pixhawk import PixhawkController
    
    # Dummy radio class for testing
    class DummyRadio:
        def send_message(self, msg): pass
        
    dummy_event = threading.Event()
    dummy_event.set()
    
    fc = PixhawkController()
    threading.Thread(target=fc.telemetry_loop, args=(dummy_event,), daemon=True).start()
    
    surveillance_loop(fc, dummy_event, DummyRadio())
