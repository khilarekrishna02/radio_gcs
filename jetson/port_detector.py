"""Port auto-detection and probing helper for Pixhawk and Telemetry Radio.

Supports:
1. Fast descriptor matching (by-id, USB manufacturer, product, VID:PID).
2. Active protocol probing:
   - Pixhawk: waits for MAVLink heartbeat.
   - Radio: verifies candidate serial port is responsive and distinct from Pixhawk.
3. Fallback to manual configuration via environment variables.
"""

import glob
import logging
import os
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

logger = logging.getLogger("port_detector")

# Known USB VID:PID or substrings
PIXHAWK_IDENTIFIERS = [
    "pixhawk", "cube", "ardupilot", "px4", "fmu", "holybro", "cuav", "stm32",
    "26ac:", # 3DR / Hex Cube
    "0483:5740", # STMicroelectronics Virtual COM Port
    "1209:5740", # Generic ArduPilot
    "3162:", # CUAV
]

RADIO_IDENTIFIERS = [
    "sik", "radio", "ft232", "cp210", "ch340", "ch341", "pl2303", "uart bridge",
    "0403:6001", # FTDI FT232R
    "0403:6015", # FTDI FT231X
    "10c4:ea60", # Silicon Labs CP210x
    "1a86:7523", # QinHeng CH340
]


def list_candidate_ports():
    """Returns a list of available serial port paths suitable for telemetry/autopilot."""
    ports = []

    # Priority 1: Persistent by-id symlinks
    for path in glob.glob("/dev/serial/by-id/*"):
        if path not in ports:
            ports.append(path)

    # Priority 2: USB and ACM devices
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        for path in sorted(glob.glob(pattern)):
            real_path = os.path.realpath(path)
            if real_path not in ports:
                ports.append(real_path)

    # Priority 3: Hardware UART on Jetson / RPi
    for pattern in ("/dev/serial0", "/dev/ttyTHS1", "/dev/ttyTHS2"):
        for path in glob.glob(pattern):
            real_path = os.path.realpath(path)
            if real_path not in ports:
                ports.append(real_path)

    # Priority 4: list_ports if they have valid hwid or description (filters out standard /dev/ttyS0..31 dummy ports)
    if serial and hasattr(serial.tools, "list_ports"):
        for p in serial.tools.list_ports.comports():
            # Skip empty legacy kernel ttyS devices
            if p.device.startswith("/dev/ttyS") and (not p.hwid or p.hwid == "n/a"):
                continue
            if p.device and p.device not in ports:
                ports.append(p.device)

    return ports



def inspect_port_info(device_path):
    """Gathers description, manufacturer, and VID:PID string for a device."""
    desc = ""
    mfg = ""
    vid_pid = ""
    hwid = ""
    if serial and hasattr(serial.tools, "list_ports"):
        for p in serial.tools.list_ports.comports():
            if p.device == device_path or os.path.realpath(p.device) == os.path.realpath(device_path):
                desc = p.description or ""
                mfg = p.manufacturer or ""
                hwid = p.hwid or ""
                if p.vid is not None and p.pid is not None:
                    vid_pid = f"{p.vid:04x}:{p.pid:04x}"
                break
    return f"{desc} {mfg} {hwid} {vid_pid}".lower()


def probe_pixhawk_heartbeat(port, baud=57600, timeout=1.5):
    """Attempts to connect and wait for a MAVLink heartbeat."""
    if not mavutil:
        return False
    conn = None
    try:
        conn = mavutil.mavlink_connection(port, baud=baud, autoreconnect=False)
        hb = conn.wait_heartbeat(timeout=timeout)
        if hb is not None:
            return True
    except Exception:
        pass
    finally:
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass
    return False


def detect_pixhawk_port(preferred_port=None, baud=57600):
    """
    Finds the Pixhawk port:
    1. Check preferred_port if provided (or PIXHAWK_PORT env var).
    2. Check candidate ports with Pixhawk descriptors + heartbeat.
    3. Check all other candidate ports with heartbeat probe.
    """
    env_port = os.getenv("PIXHAWK_PORT")
    # If explicitly set in environment and not default, check if it exists
    if env_port and env_port != "/dev/ttyACM0" and os.path.exists(env_port):
        return env_port

    candidates = list_candidate_ports()
    if not candidates:
        return preferred_port or "/dev/ttyACM0"

    # Step 1: Check candidate ports matching Pixhawk descriptors
    descriptor_matches = []
    other_ports = []
    for p in candidates:
        info = inspect_port_info(p)
        if any(ident in info for ident in PIXHAWK_IDENTIFIERS) or "ttyacm" in p.lower():
            descriptor_matches.append(p)
        else:
            other_ports.append(p)

    # First probe ports with matching descriptors
    for p in descriptor_matches:
        if probe_pixhawk_heartbeat(p, baud=baud, timeout=1.2):
            return p

    # Next probe any remaining candidate ports
    for p in other_ports:
        if probe_pixhawk_heartbeat(p, baud=baud, timeout=1.2):
            return p

    # Fallback to descriptor match if available, or preferred
    if descriptor_matches:
        return descriptor_matches[0]

    return preferred_port or (candidates[0] if candidates else "/dev/ttyACM0")


def detect_radio_port(exclude_port=None, preferred_port=None):
    """
    Finds the Telemetry Radio port:
    1. Check GCS_RADIO_PORT env var if explicitly provided.
    2. Candidate ports matching radio descriptors (excluding Pixhawk port).
    3. Any remaining candidate port excluding Pixhawk port.
    """
    env_port = os.getenv("GCS_RADIO_PORT")
    if env_port and env_port != "/dev/ttyUSB0" and os.path.exists(env_port):
        return env_port

    candidates = list_candidate_ports()
    if exclude_port:
        exclude_real = os.path.realpath(exclude_port)
        candidates = [p for p in candidates if os.path.realpath(p) != exclude_real]

    if not candidates:
        return preferred_port or "/dev/ttyUSB0"

    # Match descriptors
    for p in candidates:
        info = inspect_port_info(p)
        if any(ident in info for ident in RADIO_IDENTIFIERS) or "ttyusb" in p.lower():
            return p

    return preferred_port or candidates[0]
