# Drone Radio GCS — Lawnmower integration

This is a modular integration of the existing lawnmower workflow into the larger `drone-radio-gcs` structure. The laptop owns KML parsing and serpentine planning; the Pi owns the Pixhawk connection, mission upload and start sequence.

## Architecture

`Laptop browser -> laptop/api_proxy.py -> laptop/radio_bridge.py -> ground SiK (COM13) ~~~ air SiK (/dev/ttyUSB0 on Pi) -> rpi/radio_bridge.py -> rpi/pixhawk.py -> Pixhawk`

- The laptop generates a QGC WPL 110 `.waypoints` file.
- The file is transferred in base64 JSON chunks using stop-and-wait acknowledgements. The Pi verifies byte count and SHA-256 before accepting it.
- Clicking **Upload mission** sends `UPLOAD_REQUEST`; the Pi parses the received WPL file, uploads it to Pixhawk, and only reports success after a final accepted `MISSION_ACK` and a readback check confirms the stored item count, commands, and frames match the file.
- Clicking **Start mission** requires a browser confirmation and successful upload. The Pi checks connection/GPS, requests GUIDED, confirms mode, arms and confirms armed state, requests AUTO and confirms mode, then sends MISSION_START. The existing mission items (including TAKEOFF and RTL) are not rewritten.
- Telemetry retains `lat`, `lon`, `alt`, `mode`, `status`, `groundspeed`, `lidar_reading`, and `armed`. It is forwarded approximately once per second; it can pause briefly while Pixhawk mission protocol traffic owns the MAVLink receive lock.

## Important radio ownership rule

Only one process may open a serial port. On the Pi, `flask_app.py` starts the radio bridge as a thread, so enable **only one** of `drone-gcs.service` or `radio-bridge.service`. They intentionally run the same combined process under different unit names. Do not start both. On the laptop, only `api_proxy.py` opens the ground SiK port.

## Laptop setup (Windows example)

1. Extract this folder. Open a terminal in `drone-radio-gcs`.
2. Create an environment and install packages:

   ```powershell
   py -m venv .venv
   .\.venv\Scripts\Activate.ps1
   pip install -r laptop\requirements.txt
   ```

3. Connect the ground SiK radio and verify Windows Device Manager shows COM13 (the default in `laptop/config.py`). Close Mission Planner or any other program that might have COM13 open.
4. Start the website:

   ```powershell
   cd laptop
   $env:GCS_RADIO_PORT="COM13"
   $env:GCS_RADIO_BAUD="57600"
   python api_proxy.py
   ```

5. Open `http://127.0.0.1:5000`.

For a Linux laptop, set `GCS_RADIO_PORT=/dev/ttyUSB0` (or the actual ground radio device) and give your user permission to access the serial port.

## Raspberry Pi setup (Raspberry Pi OS / Ubuntu)

1. Copy this project to the Pi, for example `/opt/drone-radio-gcs`.
2. Connect the air SiK radio to USB. Run `ls -l /dev/ttyUSB* /dev/serial* /dev/ttyACM*` and identify the actual radio and Pixhawk device names. Do not assume `/dev/ttyUSB0` or `/dev/ttyACM0` without checking.
3. Create a virtual environment and install Pi dependencies:

   ```bash
   cd /opt/drone-radio-gcs
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r rpi/requirements.txt
   ```

4. The defaults in `rpi/config.py` match the last tested setup: `GCS_RADIO_PORT=/dev/ttyUSB0`, `PIXHAWK_PORT=/dev/ttyUSB1`, and both baud rates `57600`. These device names can change; verify with `ls -l /dev/serial/by-id/ /dev/ttyUSB* /dev/ttyACM*` and override the environment variables if needed. If your Pixhawk appears as `/dev/ttyACM0`, set `PIXHAWK_PORT=/dev/ttyACM0` and use the baud rate verified for that connection.
5. Run foreground for first-time testing:

   ```bash
   cd /opt/drone-radio-gcs/rpi
   export GCS_RADIO_PORT=/dev/ttyUSB0
   export PIXHAWK_PORT=/dev/ttyACM0
   export PIXHAWK_BAUD=115200
   python flask_app.py
   ```

6. The Pi local diagnostic API is bound to `127.0.0.1:8766` and is not exposed over the radio. The laptop receives telemetry through the SiK link.

### Optional systemd

Adjust `User`, paths, environment variables, and venv paths in the service file. Copy **one** Pi unit into `/etc/systemd/system/`, then run:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now drone-gcs.service
sudo journalctl -u drone-gcs.service -f
```

Do not enable both Pi service unit names.

## Website workflow

1. Upload KML polygon and click **Generate waypoints**.
2. Inspect/download the `.waypoints` file. Check the generated mission and altitude in a safe offline viewer or SITL.
3. Click **Send to Pi**. The UI waits for Pi-side checksum verification.
4. Click **Upload mission** and confirm. The UI waits for Pixhawk acknowledgement and verifies the stored mission by downloading it back.
5. Verify actual aircraft state, GPS, battery, geofence, flight area, and failsafes. Click **Start mission** and explicitly confirm.

## Tests (no hardware required)

From project root:

```bash
python -m unittest discover -s tests -v
```

## Safety and known limitations

- This is integration code, not flight-certified software. First validate protocol and mission upload in SITL or with propellers removed; never use a real flight as the first test.
- The radio protocol uses checksums for file integrity, but it is not encrypted or authenticated. Anyone with compatible access to the link could potentially inject messages; operate in a controlled test environment.
- The current file-transfer protocol uses stop-and-wait acknowledgements with bounded request retries and request-ID response caching. Verify retry behavior during radio bench testing before relying on it in flight.
- Mission generation uses a local tangent-plane approximation and Shapely intersections. Inspect the path around concave polygons and narrow regions. The current generator uses a single outer KML polygon ring; holes/multiple polygons are not handled.
- The generated mission includes TAKEOFF and RTL because those commands are part of the existing generator. Upload keeps the WPL home/reference row and sequence values exactly as the verified standalone uploader does; always inspect the returned readback verification result. Start sequence does not insert or alter mission items.
- The Pi checks that a GPS position exists, but the operator must independently verify GPS quality, EKF status, altitude, and safety requirements before flight.
- SiK radios are not a video link; only low-bandwidth telemetry/status and framed mission-file data are sent.
