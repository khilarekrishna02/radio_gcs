"""KML -> lawnmower scan rows -> QGroundControl WPL 110 mission file.

Adapted from the user's lawnmover_telem.py. Planning runs on the laptop only;
this module never opens a radio or Pixhawk connection.
"""
import math
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon, LineString, MultiLineString

EARTH_R = 6378137.0


def latlon_to_xy(lat, lon, lat0):
    return math.radians(lon) * EARTH_R * math.cos(math.radians(lat0)), math.radians(lat) * EARTH_R


def xy_to_latlon(x, y, lat0):
    return math.degrees(y / EARTH_R), math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))


def parse_kml_polygon(kml_path):
    tree = ET.parse(kml_path)
    root = tree.getroot()
    ns = {"kml": "http://www.opengis.net/kml/2.2"}
    node = root.find(".//kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates", ns)
    if node is None or not (node.text or "").strip():
        raise ValueError("KML does not contain a Polygon outer boundary")
    coords = []
    for token in node.text.strip().split():
        parts = token.split(",")
        if len(parts) < 2:
            continue
        lon, lat = float(parts[0]), float(parts[1])
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError(f"Invalid KML coordinate: {token}")
        coords.append((lat, lon))
    if len(coords) > 1 and coords[0] == coords[-1]:
        coords.pop()  # Shapely closes the polygon itself.
    if len(coords) < 3:
        raise ValueError("KML polygon must contain at least three distinct coordinates")
    return coords


def generate_reference_serpentine(coords_latlon, altitude=15.0, spacing=2.0,
                                  speed_mps=3.0, acceptance_radius=3.0,
                                  fence_margin=1.0, home_lat=None, home_lon=None):
    if len(coords_latlon) < 3:
        raise ValueError("At least three polygon points are required")
    if spacing <= 0 or altitude <= 0 or fence_margin < 0:
        raise ValueError("Spacing and altitude must be positive; fence margin cannot be negative")
    lat0 = sum(lat for lat, _ in coords_latlon) / len(coords_latlon)
    poly_xy = Polygon([latlon_to_xy(lat, lon, lat0) for lat, lon in coords_latlon])
    if not poly_xy.is_valid or poly_xy.area <= 0:
        raise ValueError("Invalid or zero-area KML polygon")
    safe_poly = poly_xy.buffer(-fence_margin)
    if safe_poly.is_empty:
        raise ValueError("Fence margin is too large for this polygon")
    minx, miny, maxx, maxy = safe_poly.bounds
    rows, y, direction = [], maxy - spacing / 2.0, 0
    while y >= miny:
        inter = safe_poly.intersection(LineString([(minx - 1, y), (maxx + 1, y)]))
        if not inter.is_empty:
            segments = [inter] if isinstance(inter, LineString) else list(inter.geoms) if isinstance(inter, MultiLineString) else []
            # Sort segments by x so each scanline is deterministic; alternate the overall direction.
            segments.sort(key=lambda seg: min(seg.bounds[0], seg.bounds[2]))
            if direction % 2:
                segments.reverse()
            for seg in segments:
                points = list(zip(*seg.xy))
                if direction % 2:
                    points.reverse()
                if len(points) >= 2:
                    rows.append(points)
                    direction += 1
        y -= spacing
    if not rows:
        raise RuntimeError("No scan rows generated. Reduce spacing or fence margin.")
    scan_rows = [[xy_to_latlon(x, y, lat0) for x, y in row] for row in rows]
    if home_lat is None or home_lon is None:
        # Safe planning fallback only: use the polygon's first vertex, never (0, 0).
        home_lat, home_lon = coords_latlon[0]
    mission = ["QGC WPL 110"]
    seq = 0

    def add_wp(command, lat=0.0, lon=0.0, alt=altitude, p1=0.0, p2=acceptance_radius, p3=0.0, p4=0.0):
        nonlocal seq
        mission.append(f"{seq}\t0\t3\t{command}\t{p1}\t{p2}\t{p3}\t{p4}\t{lat:.7f}\t{lon:.7f}\t{alt:.2f}\t1")
        seq += 1

    # QGC home placeholder. The Pi parser skips this row, matching the original code.
    mission.append(f"{seq}\t1\t0\t16\t0\t0\t0\t0\t{home_lat:.7f}\t{home_lon:.7f}\t{altitude:.2f}\t1")
    seq += 1
    add_wp(22, home_lat, home_lon)  # TAKEOFF
    mission.append(f"{seq}\t0\t3\t178\t0\t{speed_mps}\t-1\t0\t0\t0\t0\t1")  # DO_CHANGE_SPEED
    seq += 1
    for row in scan_rows:
        for lat, lon in row:
            add_wp(16, lat, lon)
    mission.append(f"{seq}\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1")  # RTL
    if seq > 700:
        raise RuntimeError("Mission too large for this implementation (over 700 mission rows)")
    return "\n".join(mission) + "\n", {"scan_rows": len(scan_rows), "waypoint_rows": seq, "home": [home_lat, home_lon]}


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
        # Ignore the home placeholder (sequence 0, current=1, NAV_WAYPOINT).
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
