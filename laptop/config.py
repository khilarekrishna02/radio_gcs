import os

RADIO_PORT = os.getenv("GCS_RADIO_PORT", "COM13")
RADIO_BAUD = int(os.getenv("GCS_RADIO_BAUD", "57600"))
WEB_HOST = os.getenv("GCS_WEB_HOST", "127.0.0.1")
WEB_PORT = int(os.getenv("GCS_WEB_PORT", "5000"))
MISSION_DIR = os.getenv("GCS_MISSION_DIR", os.path.join(os.path.dirname(__file__), "missions"))
RADIO_TIMEOUT = float(os.getenv("GCS_RADIO_TIMEOUT", "8"))
CHUNK_BYTES = int(os.getenv("GCS_CHUNK_BYTES", "360"))
