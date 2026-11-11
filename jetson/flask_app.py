"""Pi-side local API and process supervisor. Radio serial is opened only here via RadioBridge."""
import os
import threading
from flask import Flask, jsonify
import config
from pixhawk import PixhawkController
from radio_bridge import RadioBridge
from main_surveillance import surveillance_loop


app = Flask(__name__)
pixhawk = PixhawkController()
running = threading.Event()
running.set()
telemetry_thread = threading.Thread(target=pixhawk.telemetry_loop, args=(running,), daemon=True, name="pixhawk-telemetry")
radio = RadioBridge(pixhawk)
surveillance_thread = threading.Thread(target=surveillance_loop, args=(pixhawk, running, radio), daemon=True, name="vision-surveillance")
started = False


def start_services():
    global started
    if started:
        return
    os.makedirs(config.MISSION_DIR, exist_ok=True)
    telemetry_thread.start()
    try:
        radio.start()
    except Exception as exc:
        # The bridge keeps retrying the port in the background, so don't die at boot
        # just because the radio was not enumerated yet.
        print(f"[WARN] Pi SiK radio {config.RADIO_PORT} not ready ({exc}); retrying in background")
    surveillance_thread.start()
    started = True


@app.get("/health")
def health():
    return jsonify({"ok": True, "radio_running": radio.running.is_set(), "radio_port": config.RADIO_PORT,
                    "radio_port_open": radio.port_open,
                    "pixhawk_connected": pixhawk.connected, "pixhawk_port": config.PIXHAWK_PORT,
                    "vision_running": getattr(radio, "vision_thread", None) is not None,
                    "mission_received": bool(radio.mission_path), "mission_uploaded": pixhawk.uploaded})


@app.get("/telemetry")
def telemetry():
    return jsonify(dict(pixhawk.telemetry))


@app.get("/mission")
def mission_status():
    return jsonify({"received": bool(radio.mission_path), "uploaded": pixhawk.uploaded,
                    "mission_item_count": pixhawk.mission_item_count,
                    "filename": os.path.basename(radio.mission_path) if radio.mission_path else None})


if __name__ == "__main__":
    start_services()
    # Bind to loopback: the laptop communicates with the Pi through SiK, not this API.
    app.run(host="127.0.0.1", port=8766, debug=False, use_reloader=False, threaded=True)
