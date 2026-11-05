import os

RADIO_PORT = os.getenv("GCS_RADIO_PORT", "COM12")  # drone 1
RADIO_BAUD = int(os.getenv("GCS_RADIO_BAUD", "57600"))

RADIO2_PORT = os.getenv("GCS_RADIO_PORT", "COM12")  # drone 2
RADIO2_BAUD = int(os.getenv("GCS_RADIO_BAUD", "57600"))

WEB_HOST = os.getenv("GCS_WEB_HOST", "127.0.0.1")
WEB_PORT = int(os.getenv("GCS_WEB_PORT", "5000"))
MISSION_DIR = os.getenv("GCS_MISSION_DIR", os.path.join(os.path.dirname(__file__), "missions"))
RADIO_TIMEOUT = float(os.getenv("GCS_RADIO_TIMEOUT", "8"))
CHUNK_BYTES = int(os.getenv("GCS_CHUNK_BYTES", "360"))

# Directories
DATA_DIR = os.path.join(os.path.dirname(__file__), "data")
UPLOAD_DIR = os.path.join(DATA_DIR, "uploads")

# Survivor CSV files
MANUAL_CSV_PATH = os.path.join(
    UPLOAD_DIR, "manual_survivors.csv"
)

DRONE1_CSV_PATH = os.path.join(
    DATA_DIR, "drone1_survivors.csv"
)

# Drone 2 mission
DRONE2_MISSION_PATH = os.path.join(
    MISSION_DIR, "drone2_mission.waypoints"
)

# Create the directories automatically when the backend starts
os.makedirs(UPLOAD_DIR, exist_ok=True)
os.makedirs(MISSION_DIR, exist_ok=True)