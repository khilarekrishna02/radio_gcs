import os
import tempfile
import threading
from flask import Flask, jsonify, render_template, request, send_file
import config
from mission_planner import parse_kml_polygon, generate_reference_serpentine
from radio_bridge import RadioBridge, list_serial_ports

app = Flask(__name__, template_folder="templates")
bridge = RadioBridge()
bridge_error = None
mission_path = None
mission_meta = None
mission_lock = threading.Lock()


def ensure_radio():
    global bridge_error
    if not bridge.running:
        try:
            bridge.start()
            bridge_error = None
        except Exception as exc:
            bridge_error = str(exc)
            raise RuntimeError(f"Cannot open ground SiK radio on {config.RADIO_PORT}: {exc}")


@app.get("/")
def home():
    return render_template("index.html")


@app.get("/api/status")
def status():
    return jsonify({"radio_running": bridge.running, "radio_port": config.RADIO_PORT,
                    "radio_error": bridge_error, "telemetry": bridge.telemetry,
                    "events": bridge.events[-15:], "mission": mission_meta})


@app.get("/api/ports")
def ports():
    return jsonify(list_serial_ports())


@app.post("/api/generate")
def generate():
    global mission_path, mission_meta
    if "kml" not in request.files:
        return jsonify({"error": "Choose a KML file first."}), 400
    kml = request.files["kml"]
    if not kml.filename or not kml.filename.lower().endswith(".kml"):
        return jsonify({"error": "Upload a .kml file."}), 400
    try:
        fd, kml_path = tempfile.mkstemp(suffix=".kml")
        os.close(fd)
        kml.save(kml_path)
        coords = parse_kml_polygon(kml_path)
        telemetry = bridge.telemetry or {}
        home_lat, home_lon = telemetry.get("lat"), telemetry.get("lon")
        text, metadata = generate_reference_serpentine(
            coords,
            altitude=float(request.form.get("altitude", 15)),
            spacing=float(request.form.get("spacing", 2)),
            speed_mps=float(request.form.get("speed", 3)),
            acceptance_radius=float(request.form.get("acceptance_radius", 3)),
            fence_margin=float(request.form.get("fence_margin", 1)),
            home_lat=home_lat if home_lat is not None else None,
            home_lon=home_lon if home_lon is not None else None,
        )
        os.makedirs(config.MISSION_DIR, exist_ok=True)
        with mission_lock:
            mission_path = os.path.join(config.MISSION_DIR, "lawnmower.waypoints")
            with open(mission_path, "w", encoding="utf-8", newline="\n") as out:
                out.write(text)
            mission_meta = {**metadata, "filename": "lawnmower.waypoints", "sent_to_pi": False, "uploaded": False}
        return jsonify({"ok": True, "message": "Waypoint file generated.", "metadata": mission_meta,
                        "download_url": "/api/download-waypoints"})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 400
    finally:
        try:
            os.remove(kml_path)
        except (UnboundLocalError, OSError):
            pass


@app.get("/api/download-waypoints")
def download_waypoints():
    if not mission_path or not os.path.isfile(mission_path):
        return jsonify({"error": "Generate a mission first."}), 404
    return send_file(mission_path, as_attachment=True, download_name="lawnmower.waypoints", mimetype="text/plain")


@app.post("/api/send-mission")
def send_mission():
    global mission_meta
    if not mission_path or not os.path.isfile(mission_path):
        return jsonify({"error": "Generate a waypoint file first."}), 400
    try:
        ensure_radio()
        result = bridge.send_mission_file(mission_path, chunk_bytes=config.CHUNK_BYTES)
        mission_meta = {**(mission_meta or {}), "sent_to_pi": True, "transfer": result, "uploaded": False}
        return jsonify({"ok": True, "message": "Pi reconstructed and verified the waypoint file.", "data": result})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 502


@app.post("/api/upload-mission")
def upload_mission():
    global mission_meta
    if not mission_meta or not mission_meta.get("sent_to_pi"):
        return jsonify({"error": "Send and verify the waypoint file on the Pi first."}), 400
    try:
        ensure_radio()
        result = bridge.request("UPLOAD_REQUEST", {}, timeout=120)
        mission_meta = {**mission_meta, "uploaded": True, "upload_result": result.get("data", {})}
        return jsonify({"ok": True, "message": result.get("message", "Pixhawk acknowledged mission upload."), "data": result.get("data", {})})
    except Exception as exc:
        mission_meta = {**mission_meta, "uploaded": False}
        return jsonify({"error": str(exc)}), 502


@app.post("/api/start-mission")
def start_mission():
    if not mission_meta or not mission_meta.get("uploaded"):
        return jsonify({"error": "Mission must be uploaded and acknowledged before starting."}), 400
    try:
        ensure_radio()
        result = bridge.request("START_REQUEST", {"operator_confirmed": True}, timeout=60)
        return jsonify({"ok": True, "message": result.get("message", "Start sequence completed."), "data": result.get("data", {})})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 502


@app.post("/api/arm")
def arm():
    """Arm the Pixhawk without starting or changing the current mission."""
    try:
        ensure_radio()
        result = bridge.request("ARM_REQUEST", {"operator_confirmed": True}, timeout=15)
        data = result.get("data", {})
        return jsonify({"ok": True,
                        "message": result.get("message", "Pixhawk arm command acknowledged."),
                        "data": data})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 502


@app.post("/api/disarm")
def disarm():
    """Request normal Pixhawk disarming and wait for state confirmation."""
    try:
        ensure_radio()
        result = bridge.request("DISARM_REQUEST", {"operator_confirmed": True}, timeout=15)
        data = result.get("data", {})
        return jsonify({"ok": True,
                        "message": result.get("message", "Pixhawk disarm command acknowledged."),
                        "data": data})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 502


@app.post("/api/ping")
def ping():
    try:
        ensure_radio()
        result = bridge.request("PING", {}, timeout=5)
        return jsonify({"ok": True, "message": result.get("message", "Pi reachable.")})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 502


if __name__ == "__main__":
    os.makedirs(config.MISSION_DIR, exist_ok=True)
    try:
        ensure_radio()
    except Exception as exc:
        print(f"[WARN] {exc}. Web UI will start; set GCS_RADIO_PORT and restart when the radio is connected.")
    app.run(host=config.WEB_HOST, port=config.WEB_PORT, debug=False, use_reloader=False, threaded=True)
