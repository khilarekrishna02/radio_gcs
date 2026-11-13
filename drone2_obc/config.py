import os

RADIO_PORT = os.getenv("GCS_RADIO_PORT", "/dev/ttyUSB0")
RADIO_BAUD = int(os.getenv("GCS_RADIO_BAUD", "57600"))
# Set to /dev/ttyACM0 for USB, or /dev/serial0 for a properly configured UART adapter.
PIXHAWK_PORT = os.getenv("PIXHAWK_PORT", "/dev/ttyUSB1")
PIXHAWK_BAUD = int(os.getenv("PIXHAWK_BAUD", "57600"))
PIXHAWK_CONNECTION_MODE = os.getenv("PIXHAWK_CONNECTION_MODE", "serial")
MISSION_DIR = os.getenv("GCS_MISSION_DIR", os.path.join(os.path.dirname(__file__), "missions"))
RADIO_TIMEOUT = float(os.getenv("GCS_RADIO_TIMEOUT", "8"))
TELEMETRY_PERIOD = float(os.getenv("GCS_TELEMETRY_PERIOD", "1.0"))
LIDAR_ENABLED = os.getenv("LIDAR_ENABLED", "1").lower() not in ("0", "false", "no")
