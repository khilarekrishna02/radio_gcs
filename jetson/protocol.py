import json

PROTOCOL_VERSION = 1


def encode_message(message_type, payload=None, request_id=None):
    msg = {"v": PROTOCOL_VERSION, "type": message_type}
    if request_id is not None:
        msg["request_id"] = str(request_id)
    if payload is not None:
        msg["payload"] = payload
    return (json.dumps(msg, separators=(",", ":"), ensure_ascii=True) + "\n").encode("utf-8")


def decode_message(line):
    msg = json.loads(line.decode("utf-8") if isinstance(line, bytes) else line)
    if msg.get("v") != PROTOCOL_VERSION or not isinstance(msg.get("type"), str):
        raise ValueError("Unsupported or malformed radio message")
    return msg


def parse_qgc_wpl(text):
    """Parse QGC WPL rows using the same semantics as the verified standalone uploader.

    Every row is retained (including the WPL home/reference row), sequence numbers
    are preserved, and coordinates are represented as integer degrees * 1e7 for
    MISSION_ITEM_INT. This intentionally mirrors upload_waypoints.py.
    """
    lines = text.splitlines()
    if not lines or not lines[0].strip().startswith("QGC WPL"):
        raise ValueError("Invalid QGC WPL file header")
    items = []
    for line_no, line in enumerate(lines[1:], start=2):
        if not line.strip():
            continue
        parts = line.strip().split()
        if len(parts) != 12:
            raise ValueError(f"Line {line_no}: expected 12 fields, found {len(parts)}")
        seq, current, frame, command = map(int, parts[:4])
        items.append({
            "seq": seq,
            "current": current,
            "frame": frame,
            "command": command,
            "param1": float(parts[4]),
            "param2": float(parts[5]),
            "param3": float(parts[6]),
            "param4": float(parts[7]),
            "x": int(round(float(parts[8]) * 1e7)),
            "y": int(round(float(parts[9]) * 1e7)),
            "z": float(parts[10]),
            "autocontinue": int(parts[11]),
        })
    if not items:
        raise ValueError("No mission items found in QGC WPL file")
    for index, item in enumerate(items):
        if item["seq"] != index:
            raise ValueError(
                f"Non-contiguous WPL sequence: row index {index}, file sequence {item['seq']}"
            )
    return items
