"""Jetson-side settings. Every value can be overridden by an environment variable
(the systemd unit sets them). Keep your own rpi/config.py instead if it already
defines all of these names."""
import os

RADIO_PORT = os.getenv("GCS_RADIO_PORT", "/dev/ttyUSB0")
RADIO_BAUD = int(os.getenv("GCS_RADIO_BAUD", "57600"))
PIXHAWK_PORT = os.getenv("PIXHAWK_PORT", "/dev/ttyACM0")
PIXHAWK_BAUD = int(os.getenv("PIXHAWK_BAUD", "115200"))
PIXHAWK_CONNECTION_MODE = os.getenv("PIXHAWK_CONNECTION_MODE", "serial")   # "serial" or "udp"
LIDAR_ENABLED = os.getenv("LIDAR_ENABLED", "1") == "1"
TELEMETRY_PERIOD = float(os.getenv("GCS_TELEMETRY_PERIOD", "1.0"))
MISSION_DIR = os.getenv("GCS_MISSION_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), "missions"))
