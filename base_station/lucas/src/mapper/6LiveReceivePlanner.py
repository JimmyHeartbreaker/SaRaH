#!/usr/bin/env python3
"""
Single-script SARAH WiFi-based mapper + doorway planner.

- Runs a TCP server on port 9000
- Robot connects and sends lines: "x_mm,y_mm,rotation_deg"
  (e.g. "647.35,451.78,30.0")
- Points are converted to meters and rotated so that the FIRST rotation
  is treated as "North" (0 deg in the fixed map frame).
- A 10m x 10m occupancy grid is built around the robot.
- Multi-radius doorway detection + clustering.
- Doorways are stored in history; best doorway chosen as the one closest
  to the ultimate goal.
- A* path is planned from robot to chosen doorway (for visualisation).
- Optionally sends back "TARGET x_m y_m\n" to the robot.

You only need to run THIS script.
"""

import socket
import threading
import time
import math
import json
from collections import deque

import numpy as np
import matplotlib.pyplot as plt

# ---------------- Configuration ----------------
WIFI_HOST = "0.0.0.0"
WIFI_PORT = 9000

XY_RESOLUTION = 0.02          # 2 cm cell size
ROBOT_DIAMETER = 0.15
SAFETY_MARGIN = 0.05
DOOR_THRESHOLD = ROBOT_DIAMETER + SAFETY_MARGIN  # min gap width to count as doorway

CLUSTER_EPS = 0.25            # m, distance to merge doorway midpoints
CLUSTER_MIN_SAMPLES = 1

RING_RADII = [0.3, 0.5, 0.8, 1.2]   # m, sample multiple rings for door detection
SAMPLE_COUNT = 360

UI_PAUSE = 0.05               # seconds between UI updates
SCAN_TIME_WINDOW = 0.7        # seconds of points per "scan"
MIN_POINTS_PER_SCAN = 50      # require at least this many points

PATH_HISTORY_FILE = "path_history.json"

# Set your ultimate world goal here (meters, in fixed map frame)
# Example copied from your CSV-based code:
ULTIMATE_GOAL = (
    -5.267200469970703 / 1000.0,
    451.8113708496094 / 1000.0
)

# ---------------- Globals for shared state ----------------
shared = {
    "occupancy_map": None,     # np.array
    "center_x": None,
    "center_y": None,
    "min_x": None,
    "min_y": None,
    "x_w": None,
    "y_w": None,
    "clustered_doors": [],     # list of (gx,gy,mx,my,gap)
}
state_lock = threading.Lock()

door_history = []             # list of {"mx":..., "my":..., "gap":..., "ts":...}
visited_doors = set()         # set of (round(mx,2), round(my,2))
last_sent_target = None       # (mx,my) most recently sent to robot

wifi_conn = None              # TCP connection to robot (set by worker)
wifi_conn_lock = threading.Lock()

_initial_yaw_deg = None       # first yaw seen
_yaw_offset_rad = 0.0         # rotation to make first yaw = 0

# ---------------- Utils ----------------
def bresenham(start, end):
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)

    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    dx = x2 - x1
    dy = y2 - y1
    error = int(dx / 2.0) if dx != 0 else 0
    y_step = 1 if y1 < y2 else -1
    y = y1

    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx

    if swapped:
        points.reverse()

    return np.array(points, dtype=int)


def calc_grid_map_config(xy_resolution):
    # Fixed 10m x 10m map centered at origin
    min_x, max_x = -5.0, 5.0
    min_y, max_y = -5.0, 5.0
    xw = int(round((max_x - min_x) / xy_resolution))
    yw = int(round((max_y - min_y) / xy_resolution))
    return min_x, min_y, max_x, max_y, xw, yw


def generate_ray_casting_grid_map(ox, oy, xy_resolution):
    min_x, min_y, max_x, max_y, x_w, y_w = calc_grid_map_config(xy_resolution)
    x_w = max(1, x_w)
    y_w = max(1, y_w)

    occupancy_map = np.ones((x_w, y_w)) * 0.5
    center_x = int(round(-min_x / xy_resolution))
    center_y = int(round(-min_y / xy_resolution))

    for (x, y) in zip(ox, oy):
        ix = int(round((x - min_x) / xy_resolution))
        iy = int(round((y - min_y) / xy_resolution))
        if ix < 0 or iy < 0 or ix >= x_w or iy >= y_w:
            continue

        # Mark free along ray
        laser_pts = bresenham((center_x, center_y), (ix, iy))
        for lx, ly in laser_pts:
            if 0 <= lx < x_w and 0 <= ly < y_w:
                occupancy_map[lx, ly] = 0.0

        # Mark a small 2x2 block as occupied at hit
        for dx in range(2):
            for dy in range(2):
                nx = ix + dx
                ny = iy + dy
                if 0 <= nx < x_w and 0 <= ny < y_w:
                    occupancy_map[nx, ny] = 1.0

    return occupancy_map, center_x, center_y, min_x, min_y, x_w, y_w


def grid_to_world(min_x, min_y, ix, iy, xy_resolution):
    return min_x + ix * xy_resolution, min_y + iy * xy_resolution


def world_to_grid(min_x, min_y, x, y, xy_resolution):
    return int(round((x - min_x) / xy_resolution)), int(round((y - min_y) / xy_resolution))


def _rotate_point(x, y, theta_rad):
    ct = math.cos(theta_rad)
    st = math.sin(theta_rad)
    return x * ct - y * st, x * st + y * ct


# ---------------- Doorway detection ----------------
def detect_doorways_in_grid(
    occupancy_map,
    center_x,
    center_y,
    min_x,
    min_y,
    xy_resolution,
    radii=RING_RADII,
    sample_count=SAMPLE_COUNT,
    door_threshold=DOOR_THRESHOLD,
):
    doorways = []
    x_w, y_w = occupancy_map.shape

    for radius_m in radii:
        radius_cells = max(1, int(round(radius_m / xy_resolution)))
        angles = np.linspace(0, 2 * math.pi, sample_count, endpoint=False)
        samples = []

        for ang in angles:
            gx = int(round(center_x + radius_cells * math.cos(ang)))
            gy = int(round(center_y + radius_cells * math.sin(ang)))
            if 0 <= gx < x_w and 0 <= gy < y_w:
                samples.append((ang, gx, gy, occupancy_map[gx, gy]))
            else:
                samples.append((ang, None, None, 1.0))  # treat out-of-bounds as occupied

        segments = []
        current = []
        for ang, gx, gy, val in samples + [samples[0]]:
            if gx is not None and val < 0.4:  # free
                current.append((ang, gx, gy))
            else:
                if current:
                    segments.append(current)
                    current = []

        for seg in segments:
            if len(seg) < 2:
                continue

            ang1, ix1, iy1 = seg[0]
            ang2, ix2, iy2 = seg[-1]
            wx1, wy1 = grid_to_world(min_x, min_y, ix1, iy1, xy_resolution)
            wx2, wy2 = grid_to_world(min_x, min_y, ix2, iy2, xy_resolution)
            gap = math.hypot(wx1 - wx2, wy1 - wy2)

            if gap >= door_threshold:
                mid_idx = len(seg) // 2
                _, mxg, myg = seg[mid_idx]
                mwx, mwy = grid_to_world(min_x, min_y, mxg, myg, xy_resolution)
                doorways.append((mwx, mwy, gap, mxg, myg))

    return doorways


def cluster_points(points, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES):
    clusters = []
    members = []

    for p in points:
        added = False
        for i, c in enumerate(clusters):
            if math.hypot(p[0] - c[0], p[1] - c[1]) <= eps:
                members[i].append(p)
                xs = [q[0] for q in members[i]]
                ys = [q[1] for q in members[i]]
                clusters[i] = (sum(xs) / len(xs), sum(ys) / len(ys))
                added = True
                break
        if not added:
            clusters.append((p[0], p[1]))
            members.append([p])

    final = []
    for mem in members:
        if len(mem) >= min_samples:
            xs = [q[0] for q in mem]
            ys = [q[1] for q in mem]
            final.append((sum(xs) / len(xs), sum(ys) / len(ys)))
    return final


# ---------------- A* planner ----------------
def astar(occ_map, start, goal):
    max_x, max_y = occ_map.shape
    if not (0 <= start[0] < max_x and 0 <= start[1] < max_y):
        return []
    if not (0 <= goal[0] < max_x and 0 <= goal[1] < max_y):
        return []

    neighbors = [
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]
    move_cost = {n: math.hypot(n[0], n[1]) for n in neighbors}

    def h(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap = []
    import heapq
    start_g = 0.0
    heapq.heappush(open_heap, (start_g + h(start, goal), start_g, start))
    came_from = {start: None}
    gscore = {start: 0.0}

    while open_heap:
        _, g_curr, current = heapq.heappop(open_heap)
        if current == goal:
            break

        for dx, dy in neighbors:
            nx, ny = current[0] + dx, current[1] + dy
            if not (0 <= nx < max_x and 0 <= ny < max_y):
                continue
            if occ_map[nx, ny] >= 0.5:  # treat unknown+occupied as blocked
                continue

            tentative = g_curr + move_cost[(dx, dy)]
            neighbor = (nx, ny)
            if neighbor not in gscore or tentative < gscore[neighbor]:
                gscore[neighbor] = tentative
                fscore = tentative + h(neighbor, goal)
                heapq.heappush(open_heap, (fscore, tentative, neighbor))
                came_from[neighbor] = current

    if goal not in came_from:
        return []

    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path


# ---------------- Door history helpers ----------------
def load_history():
    global door_history
    try:
        with open(PATH_HISTORY_FILE, "r") as f:
            door_history = json.load(f)
    except Exception:
        door_history = []


def save_history():
    try:
        with open(PATH_HISTORY_FILE, "w") as f:
            json.dump(door_history, f, indent=2)
    except Exception as e:
        print("Failed to save history:", e)


def choose_best_door(clustered_doors, ultimate_goal):
    """Choose unvisited doorway closest to ultimate goal."""
    if not clustered_doors:
        return None

    gx_goal, gy_goal = ultimate_goal
    best = None
    best_dist = float("inf")

    for (gx, gy, mx, my, gap) in clustered_doors:
        key = (round(mx, 2), round(my, 2))
        if key in visited_doors:
            continue
        d = math.hypot(mx - gx_goal, my - gy_goal)
        if d < best_dist:
            best_dist = d
            best = (gx, gy, mx, my, gap)

    return best


def mark_door_visited(mx, my):
    key = (round(mx, 2), round(my, 2))
    visited_doors.add(key)


# ---------------- WiFi worker ----------------
def wifi_worker_loop():
    global wifi_conn, _initial_yaw_deg, _yaw_offset_rad, door_history

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((WIFI_HOST, WIFI_PORT))
    srv.listen(1)

    print(f"[WiFi] Listening on {WIFI_HOST}:{WIFI_PORT} ...")
    conn, addr = srv.accept()
    with wifi_conn_lock:
        wifi_conn = conn
    print(f"[WiFi] Robot connected from {addr}")

    buffer = ""
    scan_points = []
    scan_start_time = None

    while True:
        try:
            data = conn.recv(4096)
            if not data:
                print("[WiFi] Connection closed by robot.")
                break
            buffer += data.decode("utf-8", errors="ignore")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                if line.upper() == "ENDSCAN":
                    # Force processing of current scan
                    if len(scan_points) >= MIN_POINTS_PER_SCAN:
                        process_scan(scan_points)
                    scan_points = []
                    scan_start_time = None
                    continue

                parts = line.split(",")
                if len(parts) < 3:
                    # ignore malformed lines
                    continue

                try:
                    x_mm = float(parts[0])
                    y_mm = float(parts[1])
                    yaw_deg = float(parts[2])
                except ValueError:
                    continue

                # Initialise yaw reference
                if _initial_yaw_deg is None:
                    _initial_yaw_deg = yaw_deg
                    _yaw_offset_rad = math.radians(-_initial_yaw_deg)
                    print(f"[WiFi] Initial yaw = {_initial_yaw_deg:.2f} deg -> set as North")

                scan_points.append((x_mm, y_mm, yaw_deg))
                if scan_start_time is None:
                    scan_start_time = time.time()

                # Time-based scan window
                if (
                    len(scan_points) >= MIN_POINTS_PER_SCAN
                    and (time.time() - scan_start_time) >= SCAN_TIME_WINDOW
                ):
                    process_scan(scan_points)
                    scan_points = []
                    scan_start_time = None

        except Exception as e:
            print("[WiFi] Error:", e)
            break

    conn.close()
    srv.close()
    with wifi_conn_lock:
        wifi_conn = None
    print("[WiFi] Server stopped.")


def process_scan(points):
    """
    points: list of (x_mm, y_mm, yaw_deg)
    - Convert to meters
    - Rotate using yaw_rel so first yaw is North
    - Build grid / detect doorways / cluster
    - Update shared + door history
    """
    if not points:
        return

    ox = []
    oy = []

    # Use last yaw as current orientation
    yaw_deg = points[-1][2]
    yaw_rel_rad = math.radians(yaw_deg) + _yaw_offset_rad

    for x_mm, y_mm, _ in points:
        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0
        xr, yr = _rotate_point(x_m, y_m, yaw_rel_rad)
        ox.append(xr)
        oy.append(yr)

    ox = np.array(ox, dtype=float)
    oy = np.array(oy, dtype=float)

    occ_map, cx, cy, min_x, min_y, x_w, y_w = generate_ray_casting_grid_map(
        ox, oy, XY_RESOLUTION
    )

    raw_doors = detect_doorways_in_grid(
        occ_map, cx, cy, min_x, min_y, XY_RESOLUTION,
        radii=RING_RADII, sample_count=SAMPLE_COUNT,
        door_threshold=DOOR_THRESHOLD
    )

    pts = [(mwx, mwy) for (mwx, mwy, gap, gx, gy) in raw_doors]
    clusters = cluster_points(pts)
    clustered = []
    for (cxw, cyw) in clusters:
        best = None
        best_dist = float("inf")
        for (mwx, mwy, gap, gx, gy) in raw_doors:
            d = math.hypot(cxw - mwx, cyw - mwy)
            if d < best_dist:
                best = (gx, gy, mwx, mwy, gap)
                best_dist = d
        if best:
            clustered.append(best)

    # Update shared state
    with state_lock:
        shared["occupancy_map"] = occ_map
        shared["center_x"] = cx
        shared["center_y"] = cy
        shared["min_x"] = min_x
        shared["min_y"] = min_y
        shared["x_w"] = x_w
        shared["y_w"] = y_w
        shared["clustered_doors"] = clustered

    # Update door history
    if clustered:
        changed = False
        for (gx, gy, mx, my, gap) in clustered:
            rec = {"mx": mx, "my": my, "gap": gap, "ts": time.time()}
            door_history.append(rec)
            changed = True
        if changed:
            save_history()

    print(f"[WiFi] Processed scan with {len(points)} points, {len(clustered)} doors")


# ---------------- Send target to robot ----------------
def send_target_to_robot(mx, my):
    global last_sent_target
    with wifi_conn_lock:
        conn = wifi_conn
    if conn is None:
        return

    # avoid spamming same target
    if last_sent_target is not None:
        if math.isclose(last_sent_target[0], mx, abs_tol=1e-3) and \
           math.isclose(last_sent_target[1], my, abs_tol=1e-3):
            return

    msg = f"TARGET {mx:.3f} {my:.3f}\n"
    try:
        conn.sendall(msg.encode("utf-8"))
        last_sent_target = (mx, my)
        print("[WiFi] Sent target doorway:", msg.strip())
    except Exception as e:
        print("[WiFi] Failed to send TARGET:", e)


# ---------------- Main UI loop ----------------
def main():
    load_history()

    # Start WiFi worker
    worker = threading.Thread(target=wifi_worker_loop, daemon=True)
    worker.start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 6))

    while True:
        with state_lock:
            occ_map = shared["occupancy_map"]
            cx = shared["center_x"]
            cy = shared["center_y"]
            min_x = shared["min_x"]
            min_y = shared["min_y"]
            x_w = shared["x_w"]
            y_w = shared["y_w"]
            clustered_doors = list(shared["clustered_doors"])  # copy

        ax.clear()
        ax.set_title("SARAH Map / Doorways / Path")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_aspect("equal", "box")

        if occ_map is not None:
            # show free/occupied cells
            xs = []
            ys = []
            for ix in range(x_w):
                for iy in range(y_w):
                    if occ_map[ix, iy] >= 0.9:
                        wx, wy = grid_to_world(min_x, min_y, ix, iy, XY_RESOLUTION)
                        xs.append(wx)
                        ys.append(wy)
            ax.scatter(xs, ys, s=2, marker="s", alpha=0.5)

            # robot at center
            wx_r, wy_r = grid_to_world(min_x, min_y, cx, cy, XY_RESOLUTION)
            ax.scatter([wx_r], [wy_r], c="green", s=40, label="Robot")

            # doorways
            door_x = []
            door_y = []
            for (gx, gy, mx, my, gap) in clustered_doors:
                door_x.append(mx)
                door_y.append(my)
            if door_x:
                ax.scatter(door_x, door_y, c="blue", s=40, label="Detected doors")

            # ultimate goal
            ax.scatter([ULTIMATE_GOAL[0]], [ULTIMATE_GOAL[1]], c="red", s=40, label="Ultimate goal")

            # choose best doorway
            best = choose_best_door(clustered_doors, ULTIMATE_GOAL)
            if best is not None:
                gx, gy, mx, my, gap = best
                ax.scatter([mx], [my], c="yellow", s=80, edgecolors="black", label="Chosen door")

                # mark visited and send target
                mark_door_visited(mx, my)
                send_target_to_robot(mx, my)

                # A* path for visualisation
                path = astar(occ_map, (cx, cy), (gx, gy))
                if path:
                    px = []
                    py = []
                    for (ix, iy) in path:
                        wxp, wyp = grid_to_world(min_x, min_y, ix, iy, XY_RESOLUTION)
                        px.append(wxp)
                        py.append(wyp)
                    ax.plot(px, py, "m-", linewidth=2, label="A* path")

        ax.legend(loc="upper right")
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        plt.pause(UI_PAUSE)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
