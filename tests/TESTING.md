# NIDAR drone: testing guide

Two parts. **Part 1** needs only your laptop (no Jetson, no drone). **Part 2** is the step-by-step plan for when you have the Jetson, Pixhawk, camera and radios.

Rule for both parts: **do the next stage only when the current one passes.** Every stage lists what to do, what "pass" looks like, and what to check if it doesn't.

---

## Part 1: At home, laptop only

### What you are testing

The real laptop code (`api_proxy.py`, laptop `radio_bridge.py`, `index.html`) talking to the **real Jetson bridge** (`jetson/radio_bridge.py`) through a virtual radio. Only the hardware is faked (Pixhawk, camera, radio). This proves the files connect to each other and to the web page.

### 1.1 Where every file goes (repo layout)

```
repo\
  laptop\
    api_proxy.py   radio_bridge.py   start_gcs.bat   index.html
    config.py   request_manager.py   mission_planner.py
    protocol.py                  <- a copy of the full protocol.py (see below)
  jetson\
    flask_app.py   main_surveillance.py   vision_geotag.py   pixhawk.py   radio_bridge.py
    protocol.py                  <- the FULL one (has parse_qgc_wpl)
    config.py   jetson_selfcheck.py   bestest.pt (copy it here)
    systemd\drone-gcs.service
  protocol\
    message_format.md            <- documentation only; no code, nothing imports from here
  tests\
    your existing tests (test_protocol.py, test_vision_pipeline.py, test_serial_link.py)
    + virtual_radio.py  fake_jetson.py  run_home_test.bat  test_jetson_wiring.py  sample_area.kml
```

Notes:
- `laptop\radio_bridge.py` and `jetson\radio_bridge.py` are **different files** with the same name. Keep each in its own folder.
- Each machine imports `protocol.py` from **its own folder** (`from protocol import ...`), so there is nothing to put on a path. Use the same full `protocol.py` in both `laptop\` and `jetson\` so they can never drift apart (the shorter `protocol_1.py` also works on the laptop, but it must be named `protocol.py`).
- `index.html` can sit next to `api_proxy.py`, or in `templates\` or `frontend\` inside `laptop\`; the app finds it.
- `jetson\config.py` is only needed if your own Jetson config lacks the same names (it has `PIXHAWK_PORT`, `TELEMETRY_PERIOD`, and so on). Keep yours if it already has them.
- Run each script from inside its own folder.

Install once: `pip install flask pyserial shapely numpy pandas`

### 1.2 Test A: the Jetson-side files work together (automatic)

```
cd repo
python tests\test_jetson_wiring.py
```

**Pass:** the last line is `ALL JETSON WIRING TESTS PASSED`.
It checks that importing `vision_geotag` is silent, the Pixhawk buffers convert and filter correctly, a centre pixel geotags to the drone's position, 300 px right / down at 15 m gives 5 m east / south, the lidar-missing fallback works, and `main_surveillance` sends TARGET messages at most once a second per person.
**If it fails:** the message names the file. Usually an old copy of one of the `jetson` files.

### 1.3 Test B: laptop, radio bridge and web page (interactive)

Double-click `tests\run_home_test.bat`. It first runs a setup check (`preflight.py`: packages, files, that the code imports, free ports) and stops with a clear **FAIL** line if something is wrong. Then three windows open (virtual radio, fake Jetson, laptop GCS) and the browser opens `http://127.0.0.1:5000`.

| # | Do this | You should see |
|---|---|---|
| 1 | Look at the top of the page | "Ground radio open ... drone link alive", pill `DRONE LINK . ALIVE`, telemetry lat 18.5293 / lon 73.8566, mode STABILIZE |
| 2 | Section 05 | "Active model on drone: bestest.pt . camera RUNNING" |
| 3 | Upload `tests\sample_area.kml`, press Generate | "Waypoint file generated", row count shown, **Execute mission** becomes enabled |
| 4 | Press **Execute mission**, confirm | Steps: Sending waypoint file, Uploading to Pixhawk, Starting camera, Arming and starting, "Mission running". The fake Jetson window prints the upload (44 items), camera STARTED and GUIDED -> ARM -> AUTO |
| 5 | Watch the map | Drone marker flies a circle, mode AUTO, altitude climbs to 15 m |
| 6 | Click "Use browser location" (or "Set laptop position on map" and click the map) | Drone <-> laptop **ground** and **straight-line** distances; straight-line is always a bit larger than ground |
| 7 | Click anywhere on the map | A pin, plus point <-> drone ground and straight-line and point <-> laptop ground, updating live as the drone moves |
| 8 | Type coordinates in section 06, press Calculate | Same numbers as step 7 for that point |
| 9 | Wait ~6 s | A fake person (red marker) appears with distances from the drone and from the laptop |
| 10 | Stop the camera, then pick `yolov8s.pt` and Set model | Fake Jetson window prints it; the "Active model" line follows |
| 11 | In the **Virtual radio** window type `drop` | Link shows lost, then recovers by itself within a few seconds |
| 12 | Close the Fake Jetson window, then start it again with `python tests\fake_jetson.py` | Link comes back with no action on the laptop (this is what a Jetson reboot looks like); Execute works again |

Type `land` in the Fake Jetson window to land the fake drone.

### 1.4 What Part 1 does NOT prove

Real radio range and settings, the Pixhawk, the camera and GPU speed, geotag accuracy with your real camera mount, systemd start at boot, and flight failsafes. That is Part 2.

---

## Part 2: With the Jetson

Replace `<jetson-ip>` with your Jetson's address. Run Jetson commands over SSH. Laptop commands are in PowerShell.

### Stage 0: Prepare the Jetson (with internet)

1. Get the repo onto the Jetson as `~/radio_gcs/` (git clone, or `scp -r`) and put `bestest.pt` in `jetson/`. Make sure `jetson/protocol.py` is the full version. If you use a different location, change the `WorkingDirectory` path in `jetson/systemd/drone-gcs.service`.
2. Fix Windows line endings: `cd ~/radio_gcs/jetson && sed -i 's/\r$//' *.py systemd/*.service`
3. Download the stock models once, in that folder: `python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt'); YOLO('yolov8s.pt')"` (the Jetson cannot download them in the field).
4. Serial access: `sudo usermod -aG dialout,video,plugdev $USER`, then log out and back in.
5. Install PyTorch **built for Jetson/your JetPack** (not the plain PyPI one), then `pip install ultralytics pyserial flask pymavlink pandas`, plus `pyrealsense2` built for the Jetson.
6. Optional, for speed: `sudo nvpmodel -q` shows the power mode; `sudo jetson_clocks` pins clocks high.

### Stage 1: Software preflight

```
cd ~/radio_gcs/jetson
python3 jetson_selfcheck.py
```
**Pass:** `0 failure(s)` (warnings are OK for now). It checks the packages, CUDA, config values, that the radio and Pixhawk ports exist and are accessible, the model files, line endings, and that `import vision_geotag` is silent.
It also lists `/dev/serial/by-id/...` names. **Put those in the service file** instead of `ttyUSB0` / `ttyACM0` so they never swap between boots.

### Stage 2: Camera and model speed

```
python3 jetson_selfcheck.py --camera --model bestest.pt
```
**Pass:** camera 25 FPS or more at 1280x720 and model 8 or more inferences/s **on GPU**.
**If slower:** CPU torch (fix Stage 0 step 5), camera on a USB 2 port, or fly slower and higher. This number sets how fast you can fly: at 3 m/s and 10 fps the drone moves 0.3 m per frame.

### Stage 3: Pixhawk and vision on the bench (props OFF)

Terminal 1: `cd ~/radio_gcs/jetson && python flask_app.py`
**Expect in the log:** Pixhawk heartbeat/connected, "Initializing Vision Pipeline", "Flight Controller linked. Starting main vision loop". (A radio warning is fine for now.)

Terminal 2:
```
curl -s http://127.0.0.1:8766/health
curl -s http://127.0.0.1:8766/telemetry
```
**Pass:**
- `pixhawk_connected: true`, `vision_running: true`.
- `lat`/`lon` are real once the GPS has a fix (outdoors), `alt` is about 0 on the ground.
- Tilt the drone: `roll` / `pitch` change (radians); turn it: `yaw` changes.
- `lidar_reading` is in **centimetres**; compare with a tape measure.

**If not:** `Pixhawk is not connected` -> check `PIXHAWK_PORT`, cable and baud (115200). Lidar always 0 -> check the rangefinder is enabled in the Pixhawk and sends DISTANCE_SENSOR.

### Stage 4: Geotag accuracy (outdoors, props OFF)

This is the most important test of your own code.

1. With `flask_app.py` running, hold the drone level at a known height (read it from the lidar), camera pointing down, nose pointing **north** (`yaw` close to 0).
2. Put a person (or a clear target) on the ground **5 m east** of the point directly below the drone, at a measured spot.
3. Read the target position on the web page (or `human_detections_geotagged_*.csv`), and compare with the real position (tape plus a phone GPS pin). On the page you can click the map at the true spot and see the distance.
4. Repeat at 2 or 3 offsets and 2 heights, and also with the target **south** and **west**.

**Pass:** error is within what you can accept for a rescue drop (decide a number first, for example 3 m at 15 m height), and a target on the image's right side lands **east** when the nose faces north.
**If the direction is wrong:** the camera mount differs from `R_cam_to_body` in `vision_geotag.py` (it assumes a downward camera with the top of the image toward the nose). Fix the matrix, not the rest.
**If it is always a few metres off in one direction:** GPS error, compass calibration, or a time offset between frames and sensors.
Detection frames are saved to `~/nidar_out`; the CSVs are written in the `jetson` folder when the program stops.

### Stage 5: Real radio link, laptop <-> Jetson

1. Plug the radios in. On the laptop, set the radio's COM port in `start_gcs.bat` (`set GCS_RADIO_PORT=COMx`) and run it. Close Mission Planner / QGC if they hold that port.
2. On the Jetson, run `flask_app.py` with the by-id radio path: `GCS_RADIO_PORT=/dev/serial/by-id/<your radio> python flask_app.py`.
3. Both radios must be paired with the same baud/air settings (57600 by default here).

**Pass:**
- Page shows "drone link alive", telemetry age about 1 s, Ping works.
- Unplug the Jetson radio, the link drops, replug it, it recovers by itself. Do the same for the laptop radio.
- Walk away with the laptop: note the distance where the link goes quiet; send a mission file near the edge of range. It must either complete (checksummed) or fail cleanly, never half-apply.

### Stage 6: Mission transfer and Execute (props OFF, outdoors with GPS)

1. Generate a small mission with `sample_area.kml` moved to your test field, or your own KML. Set spacing from your camera, not the 2 m default: the swath is `2 x height x tan(HFOV/2)`, which is about 21 m at 15 m with a 70 deg lens; use roughly 60 to 70 percent of that.
2. Press the **manual buttons first**: Send, then Upload. On the Jetson: `curl -s http://127.0.0.1:8766/mission` should show `received` and `uploaded` true and the item count.
3. Test ARM / DISARM with the buttons.
4. Then **Execute mission** with props off. Keep everything clear of the motors and your finger on DISARM.

**Pass:** all steps complete, mode AUTO, the status line shows waypoints advancing. A failure shows the failing step in the page.

### Stage 7: Targets reach the laptop

Walk a person under the camera (props off, or hold the rig up). Expect on the Jetson log: `[BROADCAST] TARGET:1,lat,lon`, and on the page: a red marker plus distances from the drone and from the laptop.
**If nothing appears:** a person needs at least 3 detections (`MIN_POINTS_FILTER`) while GPS, attitude and lidar are fresh; check the log for `[ERROR]` lines.

### Stage 8: Boot and reconnect without SSH

```
sudo cp ~/radio_gcs/jetson/systemd/drone-gcs.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now drone-gcs
journalctl -u drone-gcs -f
```
**Pass:**
- `sudo reboot`, then with the laptop page already open the link goes alive by itself (allow 1 to 2 minutes for boot and model load). No SSH.
- Kill the process (`sudo systemctl kill drone-gcs`): it restarts in a few seconds.
- Unplug and replug the radio and the Pixhawk: both reconnect.
- After a Jetson reboot, **Execute mission** works again with one click.

### Stage 9: Before the first real flight

- Set the **failsafes in Mission Planner/QGC**: battery, radio/RC loss, GCS loss, RTL altitude and geofence. The code does not set them, and the Pixhawk keeps flying the mission if the radio drops.
- Keep the RC transmitter ready to take over.
- Flight order: manual hover with vision running (check logs), then a small lawnmower (about 20 m), then the full area.
- Test Execute in ArduPilot SITL first if you can.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `ImportError: cannot import name 'Geotagger'` | Old `vision_geotag.py` | Use the new one |
| `CUDA not available` | CPU-only PyTorch | Install NVIDIA's PyTorch for your JetPack |
| `Permission denied: /dev/ttyUSB0` | Not in `dialout` group | `sudo usermod -aG dialout $USER`, log in again |
| `device or resource busy` / cannot open port | Another program holds it (old service, QGC/Mission Planner, ModemManager) | `sudo fuser -v /dev/ttyUSB0`, stop it |
| Browser says "127.0.0.1 refused to connect" | The laptop GCS did not start (or the browser opened before it was ready) | Read the **Laptop GCS** window. Usual causes: `No module named 'protocol'` (copy `protocol.py` into `laptop\`), or flask / pyserial / shapely not installed. Run `python tests\preflight.py` to see it |
| Page never shows "drone link alive" | Wrong port or baud on either end, radios not paired, only one end running | Check `GCS_RADIO_PORT` on both sides; `jetson_selfcheck.py` |
| Execute stops at "Pixhawk is not connected on the drone" | Pixhawk cable/port/baud | `curl .../health` on the Jetson |
| Execute stops at "No GPS position" | No fix yet | Wait outdoors |
| RealSense "No device" / frame timeout | USB 2 port/cable, another program using it | USB 3 cable, close `realsense-viewer`, replug |
| Service `217/USER` | Wrong `User=` | Use `jetson-arsc` |
| Service `200/CHDIR` | Wrong `WorkingDirectory` | `/home/jetson-arsc/radio_gcs/jetson` |
| Service `203/EXEC` | Bad `ExecStart` path or CRLF line endings | Fix the path; `sed -i 's/\r$//' drone-gcs.service` |
| Targets land under the drone | No lidar and low height | Check the lidar; height must exceed 0.5 m |
| Geotags mirrored or rotated | Camera mount differs from `R_cam_to_body` | See Stage 4 |
| Map blank with no internet | Leaflet and map tiles load from the internet | Needs offline tiles (not done yet) |

### Where to look

- Jetson live log: `journalctl -u drone-gcs -f`  . Jetson local API: `curl http://127.0.0.1:8766/health` (also `/telemetry`, `/mission`).
- Detection frames: `~/nidar_out`. CSVs: in the folder the program ran from (`jetson/`) (`human_detections_geotagged_*.csv`, `final_locations_per_track.csv`).
- Laptop: the "Laptop GCS" window and the event log on the page.
