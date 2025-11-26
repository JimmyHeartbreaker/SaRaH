#!/usr/bin/env python3
"""
Multithreaded Grid map -> Doorway detection using CSV input (offline mode)
Everything else is same as original script.
"""

import time
import math
import signal
import sys
import threading
import json
import numpy as np
import matplotlib.pyplot as plt

# ---------------- Configuration ----------------
XY_RESOLUTION = 0.02  # meters per grid cell
EXTEND_AREA = 3.0

ROBOT_DIAMETER = 0.30
SAFETY_MARGIN = 0.05
DOOR_THRESHOLD = ROBOT_DIAMETER + SAFETY_MARGIN
MIN_VALID_RANGE = 0.05

CLUSTER_EPS = 0.25
CLUSTER_MIN_SAMPLES = 1
MIN_DOORWAY_SEPARATION = 0.7
ARRIVAL_RADIUS = 0.4
DETECTION_COOLDOWN = 12

RING_RADIUS_M = 1
SAMPLE_COUNT = 360
WORKER_SCAN_SECONDS = 1.0

UI_PAUSE = 0.05
PATH_HISTORY_FILE = "path_history.json"
ULTIMATE_GOAL = (-3.295401096343994,451.7666015625)

# ---------------- CSV reading ----------------
def read_csv_xy(file_path):
    data = np.loadtxt(file_path, delimiter=",")
    if data.shape[1] >= 2:
        x = data[:, 0]
        y = data[:, 1]
    else:
        raise ValueError("CSV file must contain at least two columns for X and Y")
    return x, y

# ---------------- Bresenham & grid map (same as before) ----------------
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
    error = int(dx / 2.0)
    y_step = 1 if y1 < y2 else -1
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:
        points.reverse()
    return np.array(points)

def calc_grid_map_config(ox, oy, xy_resolution):
    min_x = round(min(ox) - EXTEND_AREA / 2.0)
    min_y = round(min(oy) - EXTEND_AREA / 2.0)
    max_x = round(max(ox) + EXTEND_AREA / 2.0)
    max_y = round(max(oy) + EXTEND_AREA / 2.0)
    xw = int(round((max_x - min_x) / xy_resolution))
    yw = int(round((max_y - min_y) / xy_resolution))
    return min_x, min_y, max_x, max_y, xw, yw

def generate_ray_casting_grid_map(ox, oy, xy_resolution):
    min_x, min_y, max_x, max_y, x_w, y_w = calc_grid_map_config(ox, oy, xy_resolution)
    x_w = max(1, x_w)
    y_w = max(1, y_w)
    occupancy_map = np.ones((x_w, y_w)) / 2.0
    center_x = int(round(-min_x / xy_resolution))
    center_y = int(round(-min_y / xy_resolution))

    for (x, y) in zip(ox, oy):
        ix = int(round((x - min_x) / xy_resolution))
        iy = int(round((y - min_y) / xy_resolution))
        if ix < 0 or iy < 0 or ix >= x_w or iy >= y_w:
            continue
        laser_beams = bresenham((center_x, center_y), (ix, iy))
        for lx, ly in laser_beams:
            if 0 <= lx < x_w and 0 <= ly < y_w:
                occupancy_map[lx][ly] = 0.0
        for dx in range(2):
            for dy in range(2):
                nx = ix + dx
                ny = iy + dy
                if 0 <= nx < x_w and 0 <= ny < y_w:
                    occupancy_map[nx][ny] = 1.0
    return occupancy_map, center_x, center_y, min_x, min_y, x_w, y_w

# ---------------- Doorway detection, clustering, A* etc. remain exactly same ----------------
# Use all original functions: detect_doorways_in_grid, cluster_points, AStarPlanner, choose_doorway_by_goal, save_path_history

# ---------------- Shared state ----------------
shared = {
    "occupancy_map": None,
    "center_x": None,
    "center_y": None,
    "min_x": None,
    "min_y": None,
    "x_w": None,
    "y_w": None,
    "raw_candidates": [],
    "clustered_doors": [],
    "chosen_path": None,
    "waiting_at_doorway": False,
    "continue_search": True,
    "cooldown": 0
}
state_lock = threading.Lock()
visited_doors = set()
last_goal_world = None
visited_goals = set()
visited_details = dict()
path_stack = []
dead_ends = set()
path_history = []

# ---------------- Worker adapted for CSV ----------------
def lidar_worker(csv_file):
    global last_goal_world
    ox, oy = read_csv_xy(csv_file)
    while True:
        occ_map, cx, cy, min_x, min_y, x_w, y_w = generate_ray_casting_grid_map(ox, oy, XY_RESOLUTION)

        raw_candidates = detect_doorways_in_grid(occ_map, cx, cy, min_x, min_y, XY_RESOLUTION)

        raw_world = [(mwx, mwy, gap, gx, gy) for (mwx, mwy, gap, gx, gy) in raw_candidates
                     if (round(mwx,2), round(mwy,2)) not in visited_doors]

        pts = [(p[0], p[1]) for p in raw_world]
        clusters = cluster_points(pts, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES)

        clustered = []
        for (cxw, cyw) in clusters:
            best = min(raw_world, key=lambda r: math.hypot(cxw - r[0], cyw - r[1]))
            clustered.append((best[3], best[4], best[0], best[1], best[2]))

        with state_lock:
            shared["occupancy_map"] = occ_map
            shared["center_x"] = cx
            shared["center_y"] = cy
            shared["min_x"] = min_x
            shared["min_y"] = min_y
            shared["x_w"] = x_w
            shared["y_w"] = y_w
            shared["raw_candidates"] = raw_world
            shared["clustered_doors"] = clustered

        time.sleep(0.1)  # simulate worker cycle

# ---------------- Main ----------------
def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file = os.path.join(script_dir, "lidar_xy.csv")

    worker = threading.Thread(target=lidar_worker, args=(csv_file,), daemon=True)
    worker.start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(7,7))
    plt.show(block=False)

    user_next = threading.Event()
    def on_key(event):
        if event.key == 'q':
            plt.close()
            sys.exit(0)
        elif event.key == 'n':
            user_next.set()

    fig.canvas.mpl_connect('key_press_event', on_key)

    while True:
        with state_lock:
            occ_map = shared["occupancy_map"]
            cx = shared["center_x"]
            cy = shared["center_y"]
            clustered = list(shared["clustered_doors"])
        if occ_map is None:
            time.sleep(0.1)
            continue

        ax.clear()
        ax.imshow(occ_map.T, cmap="gray_r", origin="lower")
        ax.plot(cx, cy, "og")
        for (gx, gy, mx, my, gap) in clustered:
            ax.plot(gx, gy, "yo")
        plt.pause(0.1)

        user_next.clear()
        while not user_next.is_set():
            plt.pause(0.1)

        with state_lock:
            shared["occupancy_map"] = None
            shared["clustered_doors"] = []

if __name__ == "__main__":
    main()
