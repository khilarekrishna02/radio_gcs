"""Port auto-detection for laptop GCS ground radios.

Supports detecting single or multiple telemetry radios (e.g. Drone 1 and Drone 2)
across Linux (/dev/ttyUSB*, /dev/serial/by-id/*) and Windows (COM ports).
"""

import glob
import os
import serial
import serial.tools.list_ports

RADIO_IDENTIFIERS = [
    "sik", "radio", "ft232", "cp210", "ch340", "ch341", "pl2303", "uart bridge",
    "0403:6001",  # FTDI FT232R
    "0403:6015",  # FTDI FT231X
    "10c4:ea60",  # Silicon Labs CP210x
    "1a86:7523",  # QinHeng CH340
]


def list_candidate_ports():
    """Lists available ports excluding dummy system ttyS*."""
    ports = []

    # Priority 1: Persistent by-id
    for path in glob.glob("/dev/serial/by-id/*"):
        if path not in ports:
            ports.append(path)

    # Priority 2: USB / ACM serial nodes
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for path in sorted(glob.glob(pattern)):
            real_path = os.path.realpath(path)
            if real_path not in ports:
                ports.append(real_path)

    # Priority 3: pyserial comports
    try:
        for p in serial.tools.list_ports.comports():
            # Skip empty linux ttyS dummy ports
            if p.device.startswith("/dev/ttyS") and (not p.hwid or p.hwid == "n/a"):
                continue
            if p.device and p.device not in ports:
                ports.append(p.device)
    except Exception:
        pass

    return ports


def inspect_port_info(device_path):
    desc = ""
    mfg = ""
    hwid = ""
    vid_pid = ""
    try:
        for p in serial.tools.list_ports.comports():
            if p.device == device_path or os.path.realpath(p.device) == os.path.realpath(device_path):
                desc = p.description or ""
                mfg = p.manufacturer or ""
                hwid = p.hwid or ""
                if p.vid is not None and p.pid is not None:
                    vid_pid = f"{p.vid:04x}:{p.pid:04x}"
                break
    except Exception:
        pass
    return f"{desc} {mfg} {hwid} {vid_pid}".lower()


def detect_radio_port(exclude_ports=None, preferred_port=None):
    """
    Finds a candidate telemetry radio port.
    Excludes any ports in exclude_ports (e.g., if already assigned to Drone 1).
    """
    if exclude_ports is None:
        exclude_ports = []
    exclude_resolved = [os.path.realpath(p) for p in exclude_ports if p]

    candidates = [p for p in list_candidate_ports() if os.path.realpath(p) not in exclude_resolved]

    if not candidates:
        return preferred_port

    # 1. Match descriptors
    for p in candidates:
        info = inspect_port_info(p)
        if any(ident in info for ident in RADIO_IDENTIFIERS) or "ttyusb" in p.lower():
            return p

    # 2. Return first available candidate
    return candidates[0]
