"""Pi-side local API and process supervisor. Radio serial is opened only here via RadioBridge."""
import os
import threading
from flask import Flask, jsonify
import config
from pixhawk import PixhawkController
from radio_bridge import RadioBridge

app = Flask(__name__)
pixhawk = PixhawkController()
running = threading.Event()
running.set()
telemetry_thread = threading.Thread(target=pixhawk.telemetry_loop, args=(running,), daemon=True, name="pixhawk-telemetry")
radio = RadioBridge(pixhawk)
started = False


def start_services():
    global started
    if started:
        return
    os.makedirs(config.MISSION_DIR, exist_ok=True)
    telemetry_thread.start()
    try:
        radio.start()
        print(f"[OK] Pi SiK radio opened on {config.RADIO_PORT} @ {config.RADIO_BAUD}")
    except Exception as exc:
        print(f"[ERROR] Could not open Pi SiK radio {config.RADIO_PORT}: {exc}")
        raise
    started = True


@app.get("/health")
def health():
    return jsonify({"ok": True, "radio_running": radio.running.is_set(), "radio_port": config.RADIO_PORT,
                    "pixhawk_connected": pixhawk.connected, "pixhawk_port": config.PIXHAWK_PORT,
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
