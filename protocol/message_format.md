# Drone radio GCS message format (v1)

The SiK radios are transparent serial modems. Each side sends one UTF-8 JSON object per line (`\n`). Do not send arbitrary binary bytes directly; file chunks are base64 encoded.

Every message has `v: 1`, `type`, and where applicable `request_id` and `payload`.

## Laptop → Pi
- `PING`: check radio/application connectivity.
- `MISSION_BEGIN`: `{transfer_id, filename, size, sha256, chunks}`.
- `MISSION_CHUNK`: `{transfer_id, index, data}`; `data` is base64. Pi responds to each chunk with `ACK` or `ERROR` using the same `request_id`.
- `MISSION_END`: `{transfer_id}`; Pi verifies size and SHA-256 and replies `MISSION_RECEIVED` or `ERROR`.
- `UPLOAD_REQUEST`: Pi parses the stored QGC WPL file and uploads it to Pixhawk.
- `START_REQUEST`: start only after a successful upload and connected Pixhawk; GUIDED → arm → AUTO → mission start. Existing mission items are preserved.

## Pi → Laptop
- `RESPONSE`: `{ok, message, data}`; matched to `request_id`.
- `TELEMETRY`: latest telemetry snapshot (lat, lon, alt, mode, status, groundspeed, lidar_reading, armed).
- `EVENT`: progress/status event.

## Reliability
Each line is an independent JSON message. Mission file transfer is stop-and-wait: the laptop waits for each chunk ACK before sending the next chunk. This is deliberately conservative for a low-bandwidth SiK link. `MISSION_END` is not accepted until byte count and SHA-256 match. The radio link is not encrypted or authenticated; do not treat it as secure against a third party.


## ARM_REQUEST

Laptop-to-Pi request to arm the Pixhawk without starting the mission. Payload: `{"operator_confirmed": true}`. The Pi responds only after checking the Pixhawk heartbeat for an armed state; failure to confirm returns an error. This is a flight-control action: verify the vehicle is secured and follow the project safety checklist.
