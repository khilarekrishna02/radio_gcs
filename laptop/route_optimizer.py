
import csv
import math
import itertools

def haversine(p1, p2):
    """Calculate distance between two lat/lon points in meters"""
    R = 6371000.0
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + \
        math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def path_distance(start, path):
    """Calculate total path distance"""
    dist = 0.0
    cur = start
    for p in path:
        dist += haversine(cur, p)
        cur = p

    # last survivor to home
    dist += haversine(cur, start)
    return dist

def brute_force_tsp(home, points):
    """Solve TSP using brute force"""
    if len(points) > 10:
        print(f"[WARN] {len(points)} points is too many for brute force, using as-is")
        return path_distance(home, points), points
    
    best_dist = float("inf")
    best_path = None
    for perm in itertools.permutations(points):
        d = path_distance(home, perm)
        if d < best_dist:
            best_dist = d
            best_path = perm
    return best_dist, best_path



def load_coords_from_csv(filepath):
    """Load valid survivor coordinates and report invalid rows."""

    coords = []
    skipped_rows = []

    with open(filepath, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)

        if not reader.fieldnames:
            raise ValueError("CSV is empty or has no header.")

        required = {"Final_Lat", "Final_Lon"}
        missing = required - set(reader.fieldnames)

        if missing:
            raise ValueError(
                f"Missing CSV columns: {', '.join(missing)}"
            )

        for row_number, row in enumerate(reader, start=2):
            try:
                lat = float(row["Final_Lat"])
                lon = float(row["Final_Lon"])

                if not math.isfinite(lat) or not math.isfinite(lon):
                    raise ValueError("Non-finite coordinates")

                if not (-90 <= lat <= 90):
                    raise ValueError("Invalid latitude")

                if not (-180 <= lon <= 180):
                    raise ValueError("Invalid longitude")

                coords.append((lat, lon))

            except (TypeError, ValueError) as e:
                skipped_rows.append({
                    "row": row_number,
                    "reason": str(e)
                })

    if not coords:
        raise ValueError("No valid survivor coordinates found.")

    return coords, skipped_rows



def generate_waypoints(home, path, altitude=15, hold_time=3):
    """Generate QGC waypoint file"""
    lines = ["QGC WPL 110"]
    seq = 0
    home_lat, home_lon = home
    
    # HOME
    lines.append(f"{seq}\t1\t0\t16\t0\t0\t0\t0\t{home_lat:.7f}\t{home_lon:.7f}\t{altitude}\t1")
    seq += 1
    
    # TAKEOFF
    lines.append(f"{seq}\t0\t3\t22\t0\t0\t0\t0\t{home_lat:.7f}\t{home_lon:.7f}\t{altitude}\t1")
    seq += 1
    
    # WAYPOINTS
    for lat, lon in path:
        lines.append(
            f"{seq}\t0\t3\t16\t{hold_time}\t0\t0\t0\t{lat:.7f}\t{lon:.7f}\t{altitude}\t1"
        )
        seq += 1
    
    # RTL
    lines.append(
        f"{seq}\t0\t3\t20\t0\t0\t0\t0\t{home_lat:.7f}\t{home_lon:.7f}\t{altitude}\t1"
    )
    
    return "\n".join(lines)
