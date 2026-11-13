# Drone radio GCS message format (v1)

The SiK radios are transparent serial modems. Each side sends one UTF-8 JSON object per line (`\n`). Do not send arbitrary binary bytes directly; file chunks are base64 encoded.

Every message has `v: 1`, `type`, and where applicable `request_id` and `payload`.

"Pi" in this document means the onboard computer on the drone (currently a Jetson).

Every request is answered with a `RESPONSE` carrying the same `request_id` (a repeated request gets the stored response again). An unknown request type or a failed action returns `ok: false` with the reason in `message`.

## Laptop → Pi
- `PING`: check radio/application connectivity. Pi replies with `RESPONSE`, `data: {"pixhawk_connected": bool}`.
- `MISSION_BEGIN`: `{transfer_id, filename, size, sha256, chunks}`. `size` must be 1 to 2,000,000 bytes, `chunks` at least 1, and `sha256` a 64-character hex digest. Pi replies with `RESPONSE`, `data: {transfer_id, expected_chunks}`.
- `MISSION_CHUNK`: `{transfer_id, index, data}`; `data` is base64 and decodes to at most 1024 bytes (the laptop sends 360 by default). Sending an identical chunk again is accepted; a conflicting duplicate is rejected. Pi replies to each chunk with a `RESPONSE` using the same `request_id`: `ok: true` (data `{index, received_chunks, total_chunks}`) or `ok: false` with the reason in `message`.
- `MISSION_END`: `{transfer_id}`; Pi verifies size and SHA-256, parses the file, and replies with a `RESPONSE`: `ok: true` (data `{filename, size, sha256, mission_items}`) once the file is reconstructed and verified, or `ok: false` with the reason in `message`.
- `UPLOAD_REQUEST`: Pi parses the stored QGC WPL file and uploads it to Pixhawk. Progress is reported with `EVENT` messages (`progress`, `total`); the final `RESPONSE` follows when the Pixhawk acknowledges the mission.
- `START_REQUEST`: payload `{"operator_confirmed": true}` (required). Start only after a successful upload and connected Pixhawk; GUIDED → arm → AUTO → mission start. Existing mission items are preserved.
- `VISION_START`: payload `{}`. Start (resume) the camera and detection. Pi replies with `RESPONSE`, `data: {"camera_running": true}`. Safe to repeat.
- `VISION_STOP`: payload `{}`. Pause the camera and detection. Pi replies with `RESPONSE`, `data: {"camera_running": false}`.
- `VISION_MODEL`: payload `{"model": "<name>.pt"}`. Switch the YOLO model. File name only (no path) and it must end in `.pt`; the file must already exist on the Pi. Pi replies with `RESPONSE`, `data: {"model": "<name>.pt"}`. Detection pauses for a few seconds while the model loads, so change models on the ground.

## Pi → Laptop
- `RESPONSE`: `{ok, message, data}`; matched to `request_id`.
- `TELEMETRY`: latest telemetry snapshot (lat, lon, alt, mode, status, groundspeed, lidar_reading, armed). Sent every `TELEMETRY_PERIOD` seconds (default 1.0). Units: `lat`/`lon` degrees; `alt` metres above the launch point (not sea level); `groundspeed` m/s; `lidar_reading` raw sensor value in centimetres; `armed` is `"ARMED"` or `"DISARMED"`. It also includes `roll`, `pitch`, `yaw` (radians) and the sample times `gps_time`, `att_time` (Unix seconds). Once the vision pipeline is running it also carries `vision: {"camera_running": bool, "model": "<name>.pt"}`, which the GCS page uses to show the active model and camera state.
- `TARGET`: payload `{"pid": int, "lat": float, "lon": float, "time": float}`. A geotagged person detected by the camera. No `request_id`, no reply. See [TARGET](#target).
- `EVENT`: progress/status event. Payload `{message, level, time}` plus optional `progress` and `total`.

## Reliability
Each line is an independent JSON message. Mission file transfer is stop-and-wait: the laptop waits for each chunk's successful `RESPONSE` before sending the next chunk. This is deliberately conservative for a low-bandwidth SiK link. `MISSION_END` is not accepted until byte count and SHA-256 match. The radio link is not encrypted or authenticated; do not treat it as secure against a third party.

Retries: the laptop sends a request up to 3 times with the same `request_id` if no `RESPONSE` arrives in time. The Pi remembers its most recent 200 responses by `request_id` and re-sends the stored one for a repeated request, so a retry never performs the action twice. Pi-to-laptop messages that are not requests (`TELEMETRY`, `EVENT`, `TARGET`) are fire-and-forget.


## ARM_REQUEST

Laptop-to-Pi request to arm the Pixhawk without starting the mission. Payload: `{"operator_confirmed": true}`. The Pi responds only after checking the Pixhawk heartbeat for an armed state; failure to confirm returns an error. This is a flight-control action: verify the vehicle is secured and follow the project safety checklist.


## TARGET

Pi-to-laptop notification, one per detected person (`pid`).

- `lat`, `lon`: filtered ground position in degrees. `time`: Unix seconds when it was sent.
- Sent only once a person has at least 3 supporting detections (`MIN_POINTS_FILTER`), so a single noisy fix is never reported.
- At most one message per second per `pid`.
- Every confirmed target is re-sent every 10 seconds, because a lost message would otherwise never be repeated once the person leaves the camera view.
- The laptop keeps only the latest position per `pid` (`GET /api/targets`).

## Vision control replaces the old raw commands

The vision requests above are normal request/response messages, retried by the laptop like every other request. The earlier raw text lines `CMD:VISION_START`, `CMD:VISION_STOP` and `CMD:MODEL,<name>`, and the MAVLink STATUSTEXT `TARGET: lat, lon` sent to the Pixhawk, are no longer used.

## DISARM_REQUEST

Laptop-to-Pi request to disarm the Pixhawk normally, without changing the mission. Payload: `{"operator_confirmed": true}`; without it the Pi returns an error. The Pi responds only after the Pixhawk heartbeat confirms the disarmed state. This is a flight-control action: do not disarm a vehicle that is in the air.
