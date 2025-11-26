#!/usr/bin/env python3
"""
Offline CSV-based LiDAR simulation (multi-scan)
Press 'n' to load next CSV scan and continue exploration.
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
XY_RESOLUTION = 0.02  # meters per grid cell (2 cm)
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

# ✅ Offline CSV mode
USE_CSV_FILE = True
CSV_FOLDER = "scans"  # Folder containing scan_1.csv, scan_2.csv, ...
ULTIMATE_GOAL = (-3.295401096343994,451.7666015625)

# ---------------- Bresenham algorithm ----------------
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

# ---------------- Grid map helpers ----------------
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

# ---------------- Doorway detection ----------------
def grid_to_world(min_x, min_y, ix, iy, xy_resolution):
    x = min_x + ix * xy_resolution
    y = min_y + iy * xy_resolution
    return x, y

def detect_doorways_in_grid(occupancy_map, center_x, center_y, min_x, min_y, xy_resolution,
                            radius_m=RING_RADIUS_M, sample_count=SAMPLE_COUNT, door_threshold=DOOR_THRESHOLD):
    doorways = []
    x_w, y_w = occupancy_map.shape
    radius_cells = max(1, int(round(radius_m / xy_resolution)))
    angles = np.linspace(0, 2*math.pi, sample_count, endpoint=False)
    samples = []
    for ang in angles:
        gx = int(round(center_x + radius_cells * math.cos(ang)))
        gy = int(round(center_y + radius_cells * math.sin(ang)))
        if 0 <= gx < x_w and 0 <= gy < y_w:
            samples.append((ang, gx, gy, occupancy_map[gx][gy]))
        else:
            samples.append((ang, None, None, 1.0))

    segments = []
    current = []
    for ang, gx, gy, val in samples + [samples[0]]:
        if gx is not None and val < 0.4:
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

# ---------------- Clustering ----------------
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
    for i, mem in enumerate(members):
        if len(mem) >= min_samples:
            xs = [q[0] for q in mem]
            ys = [q[1] for q in mem]
            final.append((sum(xs)/len(xs), sum(ys)/len(ys)))
    return final

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
    "clustered_doors": []
}
state_lock = threading.Lock()

# ---------------- CSV Worker ----------------
def csv_worker_loop(csv_folder, user_next_event):
    """
    Loads CSV scans one by one each time the user presses 'n'.
    Each file should be named scan_1.csv, scan_2.csv, etc.
    """
    scan_index = 1
    while True:
        csv_path = os.path.join(csv_folder, f"scan_{scan_index}.csv")
        if not os.path.exists(csv_path):
            print(f"No more scans found after {scan_index-1} files. Waiting for 'n' to retry...")
            user_next_event.clear()
            user_next_event.wait()
            continue

        print(f"\nLoading scan {scan_index}: {csv_path}")
        with open(csv_path, newline='') as f:
            reader = csv.reader(f)
            # convert mm → meters to avoid giant grid sizes
            points = [(float(r[0]) / 1000.0, float(r[1]) / 1000.0) for r in reader if len(r) >= 2]


        ox = np.array([p[0] for p in points])
        oy = np.array([p[1] for p in points])

        occ_map, cx, cy, min_x, min_y, x_w, y_w = generate_ray_casting_grid_map(ox, oy, XY_RESOLUTION)
        raw_candidates = detect_doorways_in_grid(occ_map, cx, cy, min_x, min_y, XY_RESOLUTION)

        pts = [(mwx, mwy) for (mwx, mwy, gap, gx, gy) in raw_candidates]
        clusters = cluster_points(pts, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES)
        clustered = []
        for (cxw, cyw) in clusters:
            best = None
            best_dist = float('inf')
            for (mwx, mwy, gap, gx, gy) in raw_candidates:
                d = math.hypot(cxw - mwx, cyw - mwy)
                if d < best_dist:
                    best_dist = d
                    best = (gx, gy, mwx, mwy, gap)
            if best:
                clustered.append(best)

        with state_lock:
            shared["occupancy_map"] = occ_map
            shared["center_x"] = cx
            shared["center_y"] = cy
            shared["min_x"] = min_x
            shared["min_y"] = min_y
            shared["x_w"] = x_w
            shared["y_w"] = y_w
            shared["raw_candidates"] = raw_candidates
            shared["clustered_doors"] = clustered

        print(f"Scan {scan_index} loaded. ({len(clustered)} doors detected.)")

        # Wait for next 'n' before loading next CSV
        user_next_event.clear()
        user_next_event.wait()
        scan_index += 1

# ---------------- Main ----------------
def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    user_next = threading.Event()

    if USE_CSV_FILE:
        print("CSV mode enabled. Loading scans from:", CSV_FOLDER)
        worker = threading.Thread(target=csv_worker_loop, args=(CSV_FOLDER, user_next), daemon=True)
        worker.start()
    else:
        print("Error: CSV mode required for this version.")
        return

    plt.ion()
    fig, ax = plt.subplots(figsize=(7,7))
    plt.show(block=False)

    def on_key(event):
        if event.key == 'q':
            print("Quit requested.")
            plt.close()
            sys.exit(0)
        elif event.key == 'n':
            print("Next scan requested.")
            user_next.set()

    fig.canvas.mpl_connect('key_press_event', on_key)

    print("\nControls: 'n' = next CSV scan | 'q' = quit\n")

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
        ax.plot(cx, cy, "og", label="Robot")
        for (gx, gy, mx, my, gap) in clustered:
            ax.plot(gx, gy, "yo", markersize=6)
            ax.text(gx+1, gy+1, f"{gap:.2f}m", fontsize=6, color='y')
        ax.legend(loc='upper right')
        plt.pause(UI_PAUSE)

        time.sleep(0.2)

if __name__ == "__main__":
    main()
