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
    lines = text.splitlines()
    if not lines or not lines[0].strip().startswith("QGC WPL"):
        raise ValueError("Invalid QGC WPL file header")
    items = []
    for line in lines[1:]:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 12:
            raise ValueError(f"Invalid waypoint row: {line}")
        seq, current, frame, command = map(int, parts[:4])
        if seq == 0 and current == 1 and command == 16:
            continue
        items.append({"seq": len(items), "current": 0, "frame": frame, "command": command,
                      "param1": float(parts[4]), "param2": float(parts[5]), "param3": float(parts[6]),
                      "param4": float(parts[7]), "x": round(float(parts[8]) * 1e7),
                      "y": round(float(parts[9]) * 1e7), "z": float(parts[10]),
                      "autocontinue": int(parts[11])})
    if not items:
        raise ValueError("No mission items found in QGC WPL file")
    return items
