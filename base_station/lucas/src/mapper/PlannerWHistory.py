#!/usr/bin/env python3
"""
Improved CSV-based LiDAR simulation with:
    • Multi-radius doorway detection
    • Doorway history memory
    • Cooldown-based fallback logic
    • Smart goal selection (closest to ultimate goal)
"""

import time
import math
import signal
import sys
import threading
import json
import os
import csv
from collections import deque

import numpy as np
import matplotlib.pyplot as plt

# ---------------- Configuration ----------------
XY_RESOLUTION = 0.02
EXTEND_AREA = 3.0
ROBOT_DIAMETER = 0.15
SAFETY_MARGIN = 0.05
DOOR_THRESHOLD = ROBOT_DIAMETER + SAFETY_MARGIN

CLUSTER_EPS = 0.25
CLUSTER_MIN_SAMPLES = 1
ARRIVAL_RADIUS = 0.4
DETECTION_COOLDOWN = 12               ### NEW (now used)

RING_RADII = [0.3, 0.5, 0.8, 1.0]      ### NEW (multi-radius)
SAMPLE_COUNT = 360
UI_PAUSE = 0.05

PATH_HISTORY_FILE = "path_history.json"  ### NEW (now used)

USE_CSV_FILE = True
CSV_FOLDER = "base_station\lucas\src\mapper\scans"
ULTIMATE_GOAL = ((-5.267200469970703/1000),
                 (451.8113708496094/1000))

# ---------------- Global Door History ----------------
door_history = []                     ### NEW
last_detection_time = 0              ### NEW

# ---------------- Grid Map Code (unchanged) ----------------
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
    min_x, max_x = -5.0, 5.0
    min_y, max_y = -5.0, 5.0
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

def grid_to_world(min_x, min_y, ix, iy, xy_resolution):
    return min_x + ix * xy_resolution, min_y + iy * xy_resolution

def world_to_grid(min_x, min_y, x, y, xy_resolution):
    return int(round((x - min_x) / xy_resolution)), int(round((y - min_y) / xy_resolution))

# ---------------- Improved Doorway Detection ----------------
def detect_doorways_in_grid(
    occupancy_map, center_x, center_y,
    min_x, min_y, xy_resolution,
    radii=RING_RADII, sample_count=SAMPLE_COUNT,
    door_threshold=DOOR_THRESHOLD):

    doorways = []
    x_w, y_w = occupancy_map.shape

    ### Loop through multiple radii (NEW)
    for radius_m in radii:
        radius_cells = max(1, int(round(radius_m / xy_resolution)))
        angles = np.linspace(0, 2*math.pi, sample_count, endpoint=False)

        samples = []
        for ang in angles:
            gx = int(round(center_x + radius_cells * math.cos(ang)))
            gy = int(round(center_y + radius_cells * math.sin(ang)))

            if 0 <= gx < x_w and 0 <= gy < y_w:
                samples.append((ang, gx, gy, occupancy_map[gx][gy]))
            else:
                samples.append((ang, None, None, 1.0))  # treat out-of-bounds as wall

        # Segment detection
        segments = []
        current = []

        for ang, gx, gy, val in samples + [samples[0]]:
            if gx is not None and val < 0.4:
                current.append((ang, gx, gy))
            else:
                if current:
                    segments.append(current)
                    current = []

        # Convert segments to doorway candidates
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

# ---------------- Clustering (unchanged) ----------------
def cluster_points(points, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES):
    clusters = []
    members = []
    for p in points:
        added = False
        for i, c in enumerate(clusters):
            if math.hypot(p[0]-c[0], p[1]-c[1]) <= eps:
                members[i].append(p)
                xs = [q[0] for q in members[i]]
                ys = [q[1] for q in members[i]]
                clusters[i] = (sum(xs)/len(xs), sum(ys)/len(ys))
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
            final.append((sum(xs)/len(xs), sum(ys)/len(ys)))
    return final

# ---------------- BFS Path Planner -----------------
def bfs_path(occ_map, start, goal):
    max_x, max_y = occ_map.shape
    queue = deque([start])
    came_from = {start: None}

    while queue:
        cx, cy = queue.popleft()
        if (cx, cy) == goal:
            break

        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < max_x and 0 <= ny < max_y and occ_map[nx][ny] < 0.5 and (nx, ny) not in came_from:
                came_from[(nx, ny)] = (cx, cy)
                queue.append((nx, ny))

    if goal not in came_from:
        return []

    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from[node]
    return list(reversed(path))

# ---------------- Shared State ----------------
shared = {
    "occupancy_map": None,
    "center_x": None,
    "center_y": None,
    "min_x": None,
    "min_y": None,
    "raw_candidates": [],
    "clustered_doors": []
}
state_lock = threading.Lock()

# ---------------- CSV Worker ----------------
def csv_worker_loop(csv_folder, user_next_event):
    global door_history, last_detection_time

    scan_index = 1
    while True:
        contents = os.listdir(".")
        print(contents)
        csv_path = os.path.join(csv_folder, f"scan_{scan_index}.csv")

        if not os.path.exists(csv_path):
            print(f"No more scans after file {scan_index-1}")
            user_next_event.clear()
            user_next_event.wait()
            continue

        print(f"\nLoading scan {scan_index}: {csv_path}")

        with open(csv_path, newline='') as f:
            reader = csv.reader(f)
            header = next(reader, None)
            points = []
            for r in reader:
                if len(r) < 2:
                    continue
                try:
                    x = float(r[0]) / 1000.0
                    y = float(r[1]) / 1000.0
                    points.append((x, y))
                except:
                    continue
        mapper_displayer(points,scan_index)
        
        user_next_event.clear()
        user_next_event.wait()

        scan_index += 1

def mapper_displayer(points, scan_index):

 #CSV has been read
    ox = np.array([p[0] for p in points])
    oy = np.array([p[1] for p in points])

    occ_map, cx, cy, min_x, min_y, x_w, y_w = generate_ray_casting_grid_map(
        ox, oy, XY_RESOLUTION)

    raw_candidates = detect_doorways_in_grid(
        occ_map, cx, cy, min_x, min_y, XY_RESOLUTION)

    pts = [(mwx, mwy) for (mwx, mwy, gap, gx, gy) in raw_candidates]
    clusters = cluster_points(pts)

    clustered = []
    for (cxw, cyw) in clusters:
        best = None
        best_dist = float('inf')
        for (mwx, mwy, gap, gx, gy) in raw_candidates:
            d = math.hypot(cxw - mwx, cyw - mwy)
            if d < best_dist:
                best = (gx, gy, mwx, mwy, gap)
                best_dist = d
        if best:
            clustered.append(best)

    # ---------- Store shared state ----------
    with state_lock:
        shared["occupancy_map"] = occ_map
        shared["center_x"] = cx
        shared["center_y"] = cy
        shared["min_x"] = min_x
        shared["min_y"] = min_y
        shared["clustered_doors"] = clustered

    # ---------- Save detected doors to history (NEW) ----------
    if clustered:
        last_detection_time = time.time()

        for (gx, gy, mx, my, gap) in clustered:
            door_history.append({
                "gx": gx, "gy": gy,
                "mx": mx, "my": my,
                "gap": gap
            })

       # with open(PATH_HISTORY_FILE, "w") as f:
        #    json.dump(door_history, f, indent=2)
    
    print(f"Scan {scan_index} loaded. {len(clustered)} doors detected.")
        

# ---------------- Helper: choose the best door ----------------
def closest_door_to_goal(doors, goal_world):
    gxw, gyw = goal_world
    best = None
    best_dist = float('inf')

    for (gx, gy, mx, my, gap) in doors:
        d = math.hypot(mx - gxw, my - gyw)
        if d < best_dist:
            best_dist = d
            best = (gx, gy, mx, my, gap)

    return best

# ---------------- Main Loop ----------------
def main():
    global last_detection_time

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    user_next = threading.Event()

    print("Loading scans from:", CSV_FOLDER)
    worker = threading.Thread(
        target=csv_worker_loop, args=(CSV_FOLDER, user_next), daemon=True)
    worker.start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(7,7))
    plt.show(block=False)

    def on_key(event):
        if event.key == 'q':
            plt.close()
            sys.exit(0)
        elif event.key == 'n':
            print("Next scan requested.")
            user_next.set()

    fig.canvas.mpl_connect('key_press_event', on_key)
    print("\nControls: 'n' = next scan | 'q' = quit\n")

    while True:
        with state_lock:
            occ_map = shared["occupancy_map"]
            cx = shared["center_x"]
            cy = shared["center_y"]
            clustered = list(shared["clustered_doors"])
            min_x = shared["min_x"]
            min_y = shared["min_y"]

        if occ_map is None:
            time.sleep(0.1)
            continue

        # --------- Smart Goal Selection (NEW) -------------
        current_time = time.time()
        goal_door = None

        # Rule 1: use new detections if available
        if clustered:
            goal_door = closest_door_to_goal(clustered, ULTIMATE_GOAL)

        # Rule 2: fallback to door history after cooldown
        elif current_time - last_detection_time > DETECTION_COOLDOWN:
            if os.path.exists(PATH_HISTORY_FILE):
                with open(PATH_HISTORY_FILE) as f:
                    past = json.load(f)

                past_doors = [
                    (d["gx"], d["gy"], d["mx"], d["my"], d["gap"])
                    for d in past
                ]

                goal_door = closest_door_to_goal(past_doors, ULTIMATE_GOAL)

        # Rule 3: fallback to ultimate goal
        if goal_door:
            gx, gy, mx, my, gap = goal_door
        else:
            gx, gy = world_to_grid(min_x, min_y, *ULTIMATE_GOAL, XY_RESOLUTION)

        # Plan path
        start = (cx, cy)
        goal = (gx, gy)
        path = bfs_path(occ_map, start, goal)

        # ---------- Draw ----------
        ax.clear()
        ax.imshow(occ_map.T, cmap="gray_r", origin="lower")

        ax.plot(cx, cy, "og", label="Robot")

        for (gx, gy, mx, my, gap) in clustered:
            ax.plot(gx, gy, "yo", markersize=6)
            ax.text(gx+1, gy+1, f"{gap:.2f}m", fontsize=6, color='y')

        if path:
            px, py = zip(*path)
            ax.plot(px, py, "b-", linewidth=1.5, label="Path")

        ugx, ugy = world_to_grid(min_x, min_y, *ULTIMATE_GOAL, XY_RESOLUTION)
        ax.plot(ugx, ugy, "rx", markersize=8, label="Ultimate Goal")

        ax.legend(loc='upper right')
        plt.pause(UI_PAUSE)
        time.sleep(0.1)

if __name__ == "__main__":
    main()
