import json

PROTOCOL_VERSION = 1


def encode_message(message_type, payload=None, request_id=None):
    message = {"v": PROTOCOL_VERSION, "type": message_type}
    if request_id is not None:
        message["request_id"] = str(request_id)
    if payload is not None:
        message["payload"] = payload
    return (json.dumps(message, separators=(",", ":"), ensure_ascii=True) + "\n").encode("utf-8")


def decode_message(line):
    message = json.loads(line.decode("utf-8") if isinstance(line, bytes) else line)
    if message.get("v") != PROTOCOL_VERSION or not isinstance(message.get("type"), str):
        raise ValueError("Unsupported or malformed radio message")
    return message
