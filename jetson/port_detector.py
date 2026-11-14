
"""
Port auto-detection and probing helper for Pixhawk and Telemetry Radio.

Supports:
1. USB descriptor matching (by-id, manufacturer, product, VID:PID).
2. Pixhawk identification through MAVLink heartbeat.
3. CP2102 / CP210x USB-to-UART adapters.
4. Radio selection excluding the detected Pixhawk port.
5. Manual configuration through environment variables.
"""

import glob
import logging
import os
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


# ============================================================
# DEVICE IDENTIFIERS
# ============================================================

# Do not identify a Pixhawk only by its USB-to-serial chipset.
# CP2102 adapters can be connected to either Pixhawk or SiK radio.

PIXHAWK_IDENTIFIERS = [
    "pixhawk",
    "cube",
    "ardupilot",
    "px4",
    "fmu",
    "holybro",
    "cuav",
    "stm32",
    "26ac:",
    "0483:5740",
    "1209:5740",
    "3162:",
]

# Silicon Labs CP210x adapters are explicitly supported.
# These identifiers are used to prioritize radio candidates,
# not to prove that a device is a radio.

RADIO_IDENTIFIERS = [
    "sik",
    "radio",
    "3dr_radio",
    "ft232",
    "ch340",
    "ch341",
    "pl2303",
    "uart bridge",
    "silicon labs",
    "cp210",
    "cp2102",
    "cp2104",
    "10c4:ea60",
    "0403:6001",
    "0403:6015",
    "1a86:7523",
]


# ============================================================
# SERIAL PORT DISCOVERY
# ============================================================

def list_candidate_ports():
    """
    Returns available serial ports.

    Priority:
    1. Persistent /dev/serial/by-id paths.
    2. USB and ACM devices.
    3. Hardware UART.
    4. PySerial discovery.

    Avoids duplicate physical devices.
    """

    ports = []
    seen = set()

    def add_port(path):
        real_path = os.path.realpath(path)

        if real_path not in seen and os.path.exists(path):
            seen.add(real_path)
            ports.append(path)

    # Priority 1: Persistent by-id paths
    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        add_port(path)

    # Priority 2: USB and ACM devices
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        for path in sorted(glob.glob(pattern)):
            add_port(path)

    # Priority 3: Hardware UART
    for path in (
        "/dev/serial0",
        "/dev/ttyTHS1",
        "/dev/ttyTHS2",
    ):
        if os.path.exists(path):
            add_port(path)

    # Priority 4: PySerial discovery
    if serial and hasattr(serial.tools, "list_ports"):
        for p in serial.tools.list_ports.comports():

            # Skip empty legacy kernel serial ports
            if (
                p.device.startswith("/dev/ttyS")
                and (not p.hwid or p.hwid == "n/a")
            ):
                continue

            if p.device:
                add_port(p.device)

    logger.info("Serial candidates: %s", ports)

    return ports


# ============================================================
# DEVICE INFORMATION
# ============================================================

def inspect_port_info(device_path):
    """
    Collects:
    - Device path
    - Real device path
    - by-id symlink
    - USB description
    - Manufacturer
    - Hardware ID
    - VID:PID
    """

    info_parts = [device_path]

    real_path = os.path.realpath(device_path)

    if real_path != device_path:
        info_parts.append(real_path)

    # Check persistent by-id symlinks
    for by_id in glob.glob("/dev/serial/by-id/*"):
        if os.path.realpath(by_id) == real_path:
            info_parts.append(by_id)

    # PySerial USB information
    if serial and hasattr(serial.tools, "list_ports"):

        for p in serial.tools.list_ports.comports():

            if os.path.realpath(p.device) != real_path:
                continue

            if p.description:
                info_parts.append(p.description)

            if p.manufacturer:
                info_parts.append(p.manufacturer)

            if p.hwid:
                info_parts.append(p.hwid)

            if p.vid is not None and p.pid is not None:
                info_parts.append(
                    f"{p.vid:04x}:{p.pid:04x}"
                )

            break

    return " ".join(info_parts).lower()


# ============================================================
# PIXHAWK HEARTBEAT PROBE
# ============================================================

def probe_pixhawk_heartbeat(
    port,
    baud=57600,
    timeout=1.5
):
    """
    Checks whether a port provides a MAVLink heartbeat.

    Returns True if a heartbeat is received.
    """

    if not mavutil:
        logger.error(
            "pymavlink is not installed. Cannot probe Pixhawk."
        )
        return False

    conn = None

    try:
        logger.info(
            "Checking Pixhawk heartbeat on %s @ %s",
            port,
            baud
        )

        conn = mavutil.mavlink_connection(
            port,
            baud=baud,
            autoreconnect=False
        )

        heartbeat = conn.wait_heartbeat(timeout=timeout)

        if heartbeat is not None:
            logger.info(
                "Pixhawk heartbeat detected on %s",
                port
            )
            return True

    except Exception as exc:
        logger.debug(
            "No Pixhawk heartbeat on %s: %s",
            port,
            exc
        )

    finally:
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass

    return False


# ============================================================
# PIXHAWK PORT DETECTION
# ============================================================

def detect_pixhawk_port(
    preferred_port=None,
    baud=57600
):
    """
    Detects Pixhawk through an actual MAVLink heartbeat.

    Detection order:
    1. PIXHAWK_PORT environment variable.
    2. Preferred port.
    3. Descriptor-matching candidates.
    4. Remaining candidates.

    CP2102 is not automatically considered a Pixhawk.
    """

    env_port = os.getenv("PIXHAWK_PORT")

    # Check explicitly configured port first
    if env_port and os.path.exists(env_port):

        if probe_pixhawk_heartbeat(env_port, baud=baud):
            logger.info(
                "Using configured Pixhawk port: %s",
                env_port
            )
            return env_port

    # Check preferred port
    if preferred_port and os.path.exists(preferred_port):

        if (
            not env_port
            or os.path.realpath(preferred_port)
            != os.path.realpath(env_port)
        ):
            if probe_pixhawk_heartbeat(
                preferred_port,
                baud=baud
            ):
                return preferred_port

    candidates = list_candidate_ports()

    if not candidates:
        logger.warning("No serial ports found.")
        return preferred_port or env_port or "/dev/ttyACM0"

    descriptor_matches = []
    other_ports = []

    for port in candidates:

        info = inspect_port_info(port)

        if (
            any(
                identifier in info
                for identifier in PIXHAWK_IDENTIFIERS
            )
            or "ttyacm" in port.lower()
        ):
            descriptor_matches.append(port)

        else:
            other_ports.append(port)

    # Probe likely Pixhawk devices first
    for port in descriptor_matches:

        if port == env_port:
            continue

        if probe_pixhawk_heartbeat(port, baud=baud):
            logger.info("Pixhawk detected: %s", port)
            return port

    # Probe remaining devices
    for port in other_ports:

        if port == env_port:
            continue

        if probe_pixhawk_heartbeat(port, baud=baud):
            logger.info("Pixhawk detected: %s", port)
            return port

    logger.error(
        "Pixhawk heartbeat not detected on any candidate port."
    )

    # Do not falsely claim a descriptor match is a confirmed Pixhawk.
    return None


# ============================================================
# RADIO PORT DETECTION
# ============================================================

def detect_radio_port(
    exclude_port=None,
    preferred_port=None
):
    """
    Detects the SiK telemetry radio.

    Detection order:
    1. GCS_RADIO_PORT environment variable.
    2. Exclude the Pixhawk port.
    3. Prioritize known radio descriptors, including CP2102.
    4. Use remaining candidates as fallback.

    The descriptor match does not prove radio communication.
    """

    env_port = os.getenv("GCS_RADIO_PORT")

    exclude_resolved = set()

    if exclude_port:
        exclude_resolved.add(
            os.path.realpath(exclude_port)
        )

    # Check explicitly configured radio port
    if env_port and os.path.exists(env_port):

        if os.path.realpath(env_port) not in exclude_resolved:

            logger.info(
                "Using configured radio port: %s",
                env_port
            )

            return env_port

        logger.warning(
            "Configured radio port %s is the excluded Pixhawk port.",
            env_port
        )

    # Check preferred port
    if preferred_port and os.path.exists(preferred_port):

        if os.path.realpath(preferred_port) not in exclude_resolved:
            return preferred_port

    candidates = list_candidate_ports()

    # Remove Pixhawk from radio candidates
    filtered_candidates = [
        port
        for port in candidates
        if os.path.realpath(port) not in exclude_resolved
    ]

    if not filtered_candidates:

        logger.error("No radio candidate ports available.")

        return preferred_port or "/dev/ttyUSB0"

    radio_matches = []
    other_candidates = []

    for port in filtered_candidates:

        info = inspect_port_info(port)

        logger.info(
            "Inspecting radio candidate %s: %s",
            port,
            info
        )

        if any(
            identifier in info
            for identifier in RADIO_IDENTIFIERS
        ):
            radio_matches.append(port)

        else:
            other_candidates.append(port)

    # Prioritize CP2102, CP210x and other known USB adapters
    if radio_matches:

        selected = radio_matches[0]

        logger.info(
            "Radio candidate selected: %s",
            selected
        )

        return selected

    # Fallback
    if other_candidates:

        selected = other_candidates[0]

        logger.warning(
            "No radio descriptor match. Using fallback: %s",
            selected
        )

        return selected

    return preferred_port or "/dev/ttyUSB0"


# ============================================================
# DETECT BOTH DEVICES
# ============================================================

def detect_devices(
    pixhawk_baud=57600,
    preferred_pixhawk=None,
    preferred_radio=None
):
    """
    Detects Pixhawk first, then radio.

    Returns:
        {
            "pixhawk": "...",
            "radio": "..."
        }
    """

    logger.info("Starting serial device detection...")

    pixhawk_port = detect_pixhawk_port(
        preferred_port=preferred_pixhawk,
        baud=pixhawk_baud
    )

    if pixhawk_port is None:
        logger.error(
            "Pixhawk not detected. Radio selection will not "
            "automatically assume an unverified port is safe."
        )

        return {
            "pixhawk": None,
            "radio": None
        }

    radio_port = detect_radio_port(
        exclude_port=pixhawk_port,
        preferred_port=preferred_radio
    )

    if (
        os.path.realpath(pixhawk_port)
        == os.path.realpath(radio_port)
    ):
        logger.error(
            "Pixhawk and radio resolved to the same serial device!"
        )

        return {
            "pixhawk": None,
            "radio": None
        }

    logger.info("Device detection complete.")
    logger.info("Pixhawk: %s", pixhawk_port)
    logger.info("Radio: %s", radio_port)

    return {
        "pixhawk": pixhawk_port,
        "radio": radio_port
    }
