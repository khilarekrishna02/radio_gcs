from datetime import datetime
import math
import os
import time
import tempfile
import threading
from flask import Flask, jsonify, render_template, request, send_file
import config
from mission_planner import parse_kml_polygon, generate_reference_serpentine
from radio_bridge import RadioBridge, list_serial_ports
from route_optimizer import (
    load_coords_from_csv,
    brute_force_tsp,
    generate_waypoints
)

app = Flask(__name__, template_folder="templates")
bridge = RadioBridge(
    config.RADIO_PORT,
    config.RADIO_BAUD
)                                  # bridge for drone 1

drone2_bridge = RadioBridge(
    config.RADIO2_PORT,
    config.RADIO2_BAUD
)                                  # bridge for drone 2
bridge_errors = {
    "drone1": None,
    "drone2": None
}


#drone1
mission_path = None
mission_meta = None
mission_lock = threading.Lock()

#drone 2
drone2_mission_path = None
drone2_mission_meta = None
drone2_mission_lock = threading.Lock()

def get_drone2_home(data):
    """Get Drone 2 home coordinates from GPS, or manual fallback."""

    telemetry = drone2_bridge.telemetry or {}

    lat = telemetry.get("lat")
    lon = telemetry.get("lon")

    # Prefer valid GPS telemetry
    if lat is not None and lon is not None:
        lat = float(lat)
        lon = float(lon)

        if (
            math.isfinite(lat)
            and math.isfinite(lon)
            and -90 <= lat <= 90
            and -180 <= lon <= 180
        ):
            return (lat, lon), "gps"

    # GPS unavailable: require manual coordinates
    home_lat = data.get("home_lat")
    home_lon = data.get("home_lon")

    if home_lat is None or home_lon is None:
        raise ValueError(
            "Drone 2 GPS unavailable. Please enter home coordinates manually."
        )

    try:
        lat = float(home_lat)
        lon = float(home_lon)
    except (TypeError, ValueError):
        raise ValueError("Manual home coordinates must be valid numbers.")

    if (
        not math.isfinite(lat)
        or not math.isfinite(lon)
        or not -90 <= lat <= 90
        or not -180 <= lon <= 180
    ):
        raise ValueError("Manual home coordinates are outside valid GPS ranges.")

    return (lat, lon), "manual"

# Progress of the one-click "Execute mission" sequence (polled by the page).
# Key is "failure", not "error": the page's api() helper treats any "error" key as a failed request.
execute_state = {"running": False, "step": "idle", "failure": None, "done": False}
execute_lock = threading.Lock()


def ensure_radio(radio, drone_name):
    if not radio.running:
        try:
            radio.start()
            bridge_errors[drone_name] = None

        except Exception as exc:
            bridge_errors[drone_name] = str(exc)

            raise RuntimeError(
                f"Cannot open {drone_name} radio "
                f"on {radio.port}: {exc}"
            )

@app.route('/api/targets', methods=['GET'])
def get_detected_targets():
    """Exposes the live drone target coordinates captured by the radio bridge."""
    return jsonify(getattr(bridge, 'detected_targets', {}))

@app.get("/")
def home():
    return render_template("index.html")


@app.get("/api/status")
def status():
    return jsonify({
        "drone1": {
            "radio_running": bridge.running, 
            "radio_port": config.RADIO_PORT,
            "radio_error": bridge_errors["drone1"], 
            "telemetry": bridge.telemetry,
            "events": bridge.events[-15:], 
            "mission": mission_meta
        },
        "drone2": {
                    "radio_running": drone2_bridge.running, 
                    "radio_port": config.RADIO2_PORT,
                    "radio_error": bridge_errors["drone2"], 
                    "telemetry": drone2_bridge.telemetry,
                    "events": drone2_bridge.events[-15:], 
                    "mission": drone2_mission_meta
                },

    })

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
        ensure_radio(bridge, "drone1")
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
        ensure_radio(bridge, "drone1")
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
        ensure_radio(bridge, "drone1")
        result = bridge.request("START_REQUEST", {"operator_confirmed": True}, timeout=60)
        return jsonify({"ok": True, "message": result.get("message", "Start sequence completed."), "data": result.get("data", {})})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 502


@app.post("/api/arm")
def arm():
    """Arm the Pixhawk without starting or changing the current mission."""
    try:
        ensure_radio(bridge, "drone1")
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
        ensure_radio(bridge, "drone1")
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
        ensure_radio(bridge, "drone1")
        result = bridge.request("PING", {}, timeout=5)
        return jsonify({"ok": True, "message": result.get("message", "Pi reachable.")})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 502
@app.route('/api/vision/<action>', methods=['POST'])
def control_vision(action):
    action = action.lower()
    if action not in ("start", "stop"):
        return jsonify({"error": "Unknown vision action"}), 400
    try:
        ensure_radio(bridge, "drone1")
        result = bridge.request(f"VISION_{action.upper()}", {}, timeout=10)
        return jsonify({"message": result.get("message", f"Camera {action} command acknowledged")})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 502

@app.route('/api/vision/model', methods=['POST'])
def set_model():
    model_name = request.form.get('model', '')
    try:
        ensure_radio(bridge, "drone1")
        result = bridge.request("VISION_MODEL", {"model": model_name}, timeout=30)
        return jsonify({"message": result.get("message", f"Model update acknowledged: {model_name}")})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 502

@app.post("/api/upload-survivor-csv")
def upload_survivor_csv():
    """Upload and validate a survivor CSV for Drone 2."""

    if "file" not in request.files:
        return jsonify({"error": "No CSV file provided."}), 400

    csv_file = request.files["file"]

    if not csv_file.filename:
        return jsonify({"error": "No file selected."}), 400

    if not csv_file.filename.lower().endswith(".csv"):
        return jsonify({"error": "Please upload a CSV file."}), 400

    os.makedirs(config.UPLOAD_DIR, exist_ok=True)

    temp_path = None

    try:
        # Save temporarily so we can validate before replacing
        fd, temp_path = tempfile.mkstemp(
            suffix=".csv",
            dir=config.UPLOAD_DIR
        )
        os.close(fd)

        csv_file.save(temp_path)

        # Validate and extract coordinates
        coords, skipped_rows = load_coords_from_csv(temp_path)

        # Keep the uploaded CSV only after successful validation
        os.replace(temp_path, config.MANUAL_CSV_PATH)
        temp_path = None

        return jsonify({
            "ok": True,
            "message": "CSV uploaded and validated.",
            "source": "manual",
            "filename": "manual_survivors.csv",
            "valid_survivors": len(coords),
            "skipped_count": len(skipped_rows),
            "skipped_rows": skipped_rows,
            "requires_confirmation": len(skipped_rows) > 0
        })

    except Exception as exc:
        return jsonify({"error": str(exc)}), 400

    finally:
        if temp_path and os.path.exists(temp_path):
            os.remove(temp_path)



@app.post("/api/optimize-mission")
def optimize_drone2_mission():
    global drone2_mission_path, drone2_mission_meta

    data = request.get_json(silent=True) or {}

    # 1. Select CSV source
    source = data.get("source", "manual")

    if source == "manual":
        csv_path = config.MANUAL_CSV_PATH

    elif source == "drone1":
        csv_path = config.DRONE1_CSV_PATH

    else:
        return jsonify({
            "error": "Invalid CSV source. Use 'manual' or 'drone1'."
        }), 400

    # 2. Check CSV availability
    if not os.path.isfile(csv_path):
        return jsonify({
            "error": f"No CSV file found for source: {source}"
        }), 404

    try:
        # 3. Load and validate coordinates
        coords, skipped_rows = load_coords_from_csv(csv_path)

        # 4. Require confirmation if rows were skipped
        if skipped_rows and data.get("confirm_skipped") is not True:
            return jsonify({
                "ok": False,
                "requires_confirmation": True,
                "valid_survivors": len(coords),
                "skipped_count": len(skipped_rows),
                "skipped_rows": skipped_rows,
                "message": "Some CSV rows are invalid. Confirm before generating the mission."
            }), 409

        # 5. Get Drone 2 home coordinates
        home, home_source = get_drone2_home(data)

        # 6. Get mission parameters
        altitude = float(data.get("altitude", 15))
        hold_time = float(data.get("hold_time", 3))

        if not math.isfinite(altitude) or altitude <= 0:
            raise ValueError("Altitude must be a positive finite number.")

        if not math.isfinite(hold_time) or hold_time < 0:
            raise ValueError("Hold time must be a non-negative finite number.")

        # 7. Optimize route
        best_distance, best_path = brute_force_tsp(home, coords)

        # 8. Generate QGC waypoint mission
        mission_text = generate_waypoints(
            home,
            best_path,
            altitude=altitude,
            hold_time=hold_time
        )

        # 9. Save Drone 2 mission
        os.makedirs(config.MISSION_DIR, exist_ok=True)

        mission_path = config.DRONE2_MISSION_PATH

        with drone2_mission_lock:
            temp_path = mission_path + ".tmp"

            try:
                with open(temp_path, "w", encoding="utf-8", newline="\n") as f:
                    f.write(mission_text)

                os.replace(temp_path, mission_path)

            finally:
                if os.path.exists(temp_path):
                    os.remove(temp_path)

            drone2_mission_path = mission_path

            drone2_mission_meta = {
                "filename": os.path.basename(mission_path),
                "source": source,
                "home_source": home_source,
                "home": {
                    "lat": home[0],
                    "lon": home[1]
                },
                "total_survivors": len(coords),
                "skipped_count": len(skipped_rows),
                "skipped_rows": skipped_rows,
                "distance_m": round(best_distance, 2),
                "altitude": altitude,
                "hold_time": hold_time,
                "optimized_order": [
                    {"lat": lat, "lon": lon}
                    for lat, lon in best_path
                ],
                "generated_at": datetime.now().isoformat(),
                "sent_to_pi": False,
                "uploaded": False
            }

        return jsonify({
            "ok": True,
            "message": "Drone 2 mission generated successfully.",
            "metadata": drone2_mission_meta,
            "download_url": "/api/drone2/download-mission"
        })

    except FileNotFoundError as exc:
        return jsonify({"error": str(exc)}), 404

    except (ValueError, TypeError) as exc:
        return jsonify({"error": str(exc)}), 400

    except Exception as exc:
        app.logger.exception("Drone 2 mission optimization failed")
        return jsonify({"error": str(exc)}), 500

@app.route("/api/drone2/download-mission", methods=["GET"])
def download_drone2_mission():
    if not drone2_mission_path or not os.path.exists(drone2_mission_path):
        return jsonify({"error": "No Drone 2 mission generated"}), 404

    return send_file(
        drone2_mission_path,
        as_attachment=True,
        download_name="drone2_mission.waypoints",
        mimetype="text/plain"
    )

@app.route("/api/drone2/send-mission", methods=["POST"])
def send_drone2_mission():
    if not drone2_mission_path or not os.path.exists(drone2_mission_path):
        return jsonify({"error": "No Drone 2 mission generated"}), 404

    try:
        ensure_radio(drone2_bridge, "drone2")

        result = drone2_bridge.send_mission_file(drone2_mission_path)

        drone2_mission_meta["sent_to_pi"] = True

        return jsonify({
            "success": True,
            "message": "Mission sent to Drone 2 Jetson",
            "result": result
        })

    except Exception as exc:
        drone2_mission_meta["sent_to_pi"] = False

        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500

@app.route("/api/drone2/upload-mission", methods=["POST"])
def upload_drone2_mission():
    if not drone2_mission_path or not os.path.exists(drone2_mission_path):
        return jsonify({"error": "No generated mission found"}), 404

    if not drone2_mission_meta or not drone2_mission_meta.get("sent_to_pi"):
        return jsonify({
            "error": "Send the mission to Drone 2 Jetson first"
        }), 400

    try:
        ensure_radio(drone2_bridge, "drone2")

        result = drone2_bridge.request(
            "UPLOAD_REQUEST",
            {},
            timeout=120
        )

        drone2_mission_meta["uploaded"] = True

        return jsonify({
            "success": True,
            "message": "Mission uploaded to Pixhawk",
            "result": result
        })

    except Exception as exc:
        drone2_mission_meta["uploaded"] = False

        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500

@app.route("/api/drone2/start-mission", methods=["POST"])
def start_drone2_mission():
    if not drone2_mission_meta:
        return jsonify({"error": "No mission generated"}), 400

    if not drone2_mission_meta.get("uploaded"):
        return jsonify({
            "error": "Upload the mission to Pixhawk first"
        }), 400

    data = request.get_json(silent=True) or {}

    if data.get("operator_confirmed") is not True:
        return jsonify({
            "error": "Operator confirmation is required"
        }), 400

    try:
        ensure_radio(drone2_bridge, "drone2")

        result = drone2_bridge.request(
            "START_REQUEST",
            {"operator_confirmed": True},
            timeout=30
        )

        drone2_mission_meta["started"] = True

        return jsonify({
            "success": True,
            "message": "Drone 2 mission start command acknowledged",
            "result": result
        })

    except Exception as exc:
        drone2_mission_meta["started"] = False

        return jsonify({
            "success": False,
            "error": str(exc)
        }), 500


def _execute_worker(path):
    """Full chain every time, so a rebooted Jetson / stale flags can never leave it half-armed."""
    global mission_meta
    def step(text):
        execute_state["step"] = text
    try:
        step("Checking radio link")
        ensure_radio(bridge, "drone1")
        ping = bridge.request("PING", {}, timeout=5)
        if not (ping.get("data") or {}).get("pixhawk_connected"):
            raise RuntimeError("Pixhawk is not connected on the drone")
        tel = bridge.telemetry or {}
        if tel.get("lat") is None:
            raise RuntimeError("No GPS position from the drone yet")
        if time.time() - (tel.get("last_update") or 0) > 10:
            raise RuntimeError("No recent telemetry from the drone")

        step("Sending waypoint file to drone")
        result = bridge.send_mission_file(path, chunk_bytes=config.CHUNK_BYTES)
        mission_meta = {**(mission_meta or {}), "sent_to_pi": True, "transfer": result, "uploaded": False}

        step("Uploading mission to Pixhawk")
        up = bridge.request("UPLOAD_REQUEST", {}, timeout=120)
        mission_meta = {**mission_meta, "uploaded": True, "upload_result": up.get("data", {})}

        step("Starting camera and detection")
        bridge.request("VISION_START", {}, timeout=15)

        step("Arming and starting mission")
        bridge.request("START_REQUEST", {"operator_confirmed": True}, timeout=60)

        execute_state["done"] = True
        step("Mission running")
    except Exception as exc:
        execute_state["failure"] = f"{execute_state['step']}: {exc}"
    finally:
        execute_state["running"] = False


@app.post("/api/execute-mission")
def execute_mission():
    body = request.get_json(silent=True) or {}
    if body.get("confirmed") is not True:
        return jsonify({"error": "Operator confirmation is required"}), 400
    with mission_lock:
        path = mission_path
    if not path or not os.path.isfile(path):
        return jsonify({"error": "Generate a mission first."}), 400
    with execute_lock:
        if execute_state["running"]:
            return jsonify({"error": "A mission start is already in progress."}), 409
        execute_state.update(running=True, step="Starting", failure=None, done=False)
    threading.Thread(target=_execute_worker, args=(path,), daemon=True, name="execute-mission").start()
    return jsonify({"ok": True, "message": "Mission start sequence began."})


@app.get("/api/execute-status")
def execute_status():
    return jsonify(execute_state)


def _radio_supervisor():
    """Opens the ground radio as soon as it is plugged in (the bridge itself handles re-plugs)."""
    while True:
        if not bridge.running:
            try:
                ensure_radio(bridge, "drone1")
            except Exception:
                pass
        time.sleep(3)


if __name__ == "__main__":
    os.makedirs(config.MISSION_DIR, exist_ok=True)
    try:
        ensure_radio(bridge, "drone1")
    except Exception as exc:
        print(f"[WARN] {exc}. Web UI will start; the radio will be opened automatically once it is plugged in.")
    threading.Thread(target=_radio_supervisor, daemon=True, name="radio-supervisor").start()
    app.run(host=config.WEB_HOST, 
            port=config.WEB_PORT, 
            debug=False, 
            use_reloader=False, 
            threaded=True)
