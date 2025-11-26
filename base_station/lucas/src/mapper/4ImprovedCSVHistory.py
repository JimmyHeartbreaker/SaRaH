#!/usr/bin/env python3
"""
LiDAR CSV simulator with:
 - multi-radius doorway detection
 - persistent door history & cooldown fallback
 - door selection: closest to ultimate goal
 - visited-area overlay and unexplored-door highlighting
 - A* (8-connected) path planner
Controls:
 - 'n' to load the next scan
 - 'q' to quit
"""

import os
import sys
import time
import math
import signal
import threading
import json
import csv
from collections import deque
import heapq

import numpy as np
import matplotlib.pyplot as plt

# ---------------- Configuration ----------------
XY_RESOLUTION = 0.02           # meters per grid cell
ROBOT_DIAMETER = 0.15
SAFETY_MARGIN = 0.05
DOOR_THRESHOLD = ROBOT_DIAMETER + SAFETY_MARGIN

CLUSTER_EPS = 0.25
CLUSTER_MIN_SAMPLES = 1
DETECTION_COOLDOWN = 12.0      # seconds before we fallback to history

# Multi-radius detection (improves detection coverage)
RING_RADII = [0.3, 0.5, 0.8, 1.2]   # meters
SAMPLE_COUNT = 360
UI_PAUSE = 0.05

PATH_HISTORY_FILE = "path_history.json"

CSV_FOLDER = "scans"
ULTIMATE_GOAL = ((-5.267200469970703/1000.0),
                 (451.8113708496094/1000.0))  # in meters

# ---------------- Globals ----------------
door_history = []        # list of dicts: {"mx":..., "my":..., "gap":..., "ts":...}
last_detection_time = 0.0

# ---------------- Utilities ----------------
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
    # fixed 10m x 10m map centered at origin
    min_x, max_x = -5.0, 5.0
    min_y, max_y = -5.0, 5.0
    xw = int(round((max_x - min_x) / xy_resolution))
    yw = int(round((max_y - min_y) / xy_resolution))
    return min_x, min_y, max_x, max_y, xw, yw

def generate_ray_casting_grid_map(ox, oy, xy_resolution):
    min_x, min_y, max_x, max_y, x_w, y_w = calc_grid_map_config(xy_resolution)
    x_w = max(1, x_w); y_w = max(1, y_w)
    occupancy_map = np.ones((x_w, y_w)) * 0.5
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
                occupancy_map[lx, ly] = 0.0
        # mark a 2x2 block occupied at the hit location
        for dx in range(2):
            for dy in range(2):
                nx = ix + dx; ny = iy + dy
                if 0 <= nx < x_w and 0 <= ny < y_w:
                    occupancy_map[nx, ny] = 1.0
    return occupancy_map, center_x, center_y, min_x, min_y, x_w, y_w

def grid_to_world(min_x, min_y, ix, iy, xy_resolution):
    return min_x + ix * xy_resolution, min_y + iy * xy_resolution

def world_to_grid(min_x, min_y, x, y, xy_resolution):
    return int(round((x - min_x) / xy_resolution)), int(round((y - min_y) / xy_resolution))

# ---------------- Door detection (multi-radius) ----------------
def detect_doorways_in_grid(occupancy_map, center_x, center_y, min_x, min_y, xy_resolution,
                            radii=RING_RADII, sample_count=SAMPLE_COUNT, door_threshold=DOOR_THRESHOLD):
    doorways = []
    x_w, y_w = occupancy_map.shape

    for radius_m in radii:
        radius_cells = max(1, int(round(radius_m / xy_resolution)))
        angles = np.linspace(0, 2*math.pi, sample_count, endpoint=False)
        samples = []
        for ang in angles:
            gx = int(round(center_x + radius_cells * math.cos(ang)))
            gy = int(round(center_y + radius_cells * math.sin(ang)))
            if 0 <= gx < x_w and 0 <= gy < y_w:
                samples.append((ang, gx, gy, occupancy_map[gx, gy]))
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

# ---------------- Simple cluster (like before) ----------------
def cluster_points(points, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES):
    clusters = []
    members = []
    for p in points:
        added = False
        for i, c in enumerate(clusters):
            if math.hypot(p[0]-c[0], p[1]-c[1]) <= eps:
                members[i].append(p)
                xs = [q[0] for q in members[i]]; ys = [q[1] for q in members[i]]
                clusters[i] = (sum(xs)/len(xs), sum(ys)/len(ys))
                added = True
                break
        if not added:
            clusters.append((p[0], p[1])); members.append([p])
    final = []
    for mem in members:
        if len(mem) >= min_samples:
            xs=[q[0] for q in mem]; ys=[q[1] for q in mem]
            final.append((sum(xs)/len(xs), sum(ys)/len(ys)))
    return final

# ---------------- A* path planner (8-connected) ----------------
def astar(occ_map, start, goal):
    max_x, max_y = occ_map.shape
    if not (0 <= start[0] < max_x and 0 <= start[1] < max_y):
        return []
    if not (0 <= goal[0] < max_x and 0 <= goal[1] < max_y):
        return []

    def h(a, b):
        # Euclidean heuristic
        return math.hypot(a[0]-b[0], a[1]-b[1])

    neighbors = [ (1,0), (-1,0), (0,1), (0,-1),
                  (1,1), (1,-1), (-1,1), (-1,-1) ]
    cost = { (dx,dy): (math.hypot(dx,dy)) for dx,dy in neighbors }

    open_heap = []
    heapq.heappush(open_heap, (0 + h(start, goal), 0, start))
    came_from = {start: None}
    gscore = {start: 0}

    while open_heap:
        _, g_curr, current = heapq.heappop(open_heap)
        if current == goal:
            break
        for dx, dy in neighbors:
            nx, ny = current[0] + dx, current[1] + dy
            if not (0 <= nx < max_x and 0 <= ny < max_y): continue
            if occ_map[nx][ny] >= 0.5: continue  # blocked
            tentative = g_curr + cost[(dx,dy)]
            neighbor = (nx, ny)
            if neighbor not in gscore or tentative < gscore[neighbor]:
                gscore[neighbor] = tentative
                priority = tentative + h(neighbor, goal)
                heapq.heappush(open_heap, (priority, tentative, neighbor))
                came_from[neighbor] = current

    if goal not in came_from:
        return []
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path

# ---------------- Shared state between CSV worker and main loop ----------------
shared = {"occupancy_map": None, "center_x": None, "center_y": None,
          "min_x": None, "min_y": None, "clustered_doors": []}
state_lock = threading.Lock()

# ---------------- CSV worker ----------------
def csv_worker_loop(csv_folder, user_next_event):
    global door_history, last_detection_time
    scan_index = 1
    while True:
        csv_path = os.path.join(csv_folder, f"scan_{scan_index}.csv")
        if not os.path.exists(csv_path):
            print(f"No more scans after {scan_index-1}. Waiting for 'n' to retry or add files.")
            user_next_event.clear(); user_next_event.wait(); continue

        print(f"\nLoading scan {scan_index}: {csv_path}")
        points = []
        with open(csv_path, newline='') as f:
            reader = csv.reader(f)
            header = next(reader, None)
            for r in reader:
                if len(r) < 2: continue
                try:
                    x = float(r[0]) / 1000.0; y = float(r[1]) / 1000.0
                    points.append((x, y))
                except:
                    continue
        if not points:
            print("No valid points, skipping.")
            scan_index += 1
            user_next_event.clear(); user_next_event.wait(); continue

        ox = np.array([p[0] for p in points]); oy = np.array([p[1] for p in points])
        occ_map, cx, cy, min_x, min_y, x_w, y_w = generate_ray_casting_grid_map(ox, oy, XY_RESOLUTION)

        raw_candidates = detect_doorways_in_grid(occ_map, cx, cy, min_x, min_y, XY_RESOLUTION)

        # cluster by world midpoint
        pts = [(mwx, mwy) for (mwx, mwy, gap, gx, gy) in raw_candidates]
        clusters = cluster_points(pts)
        clustered = []
        for (cxw, cyw) in clusters:
            best = None; best_dist = float('inf')
            for (mwx, mwy, gap, gx, gy) in raw_candidates:
                d = math.hypot(cxw - mwx, cyw - mwy)
                if d < best_dist:
                    best = (gx, gy, mwx, mwy, gap); best_dist = d
            if best: clustered.append(best)

        # update shared state
        with state_lock:
            shared["occupancy_map"] = occ_map
            shared["center_x"] = cx
            shared["center_y"] = cy
            shared["min_x"] = min_x
            shared["min_y"] = min_y
            shared["clustered_doors"] = clustered

        # update persistent history if we saw doors (store world midpoints)
        if clustered:
            last_detection_time = time.time()
            changed = False
            for (gx, gy, mwx, mwy, gap) in clustered:
                rec = {"mx": mwx, "my": mwy, "gap": gap, "ts": time.time()}
                door_history.append(rec)
                changed = True
            if changed:
                try:
                    with open(PATH_HISTORY_FILE, "w") as f:
                        json.dump(door_history, f, indent=2)
                except Exception as e:
                    print("Failed to write history:", e)

        print(f"Scan {scan_index} loaded. {len(clustered)} doors detected.")
        user_next_event.clear(); user_next_event.wait()
        scan_index += 1

# ---------------- Helpers ----------------
def load_history():
    global door_history
    if os.path.exists(PATH_HISTORY_FILE):
        try:
            with open(PATH_HISTORY_FILE) as f:
                door_history = json.load(f)
        except Exception:
            door_history = []
    else:
        door_history = []

def closest_door_to_goal(doors, goal_world):
    # doors: iterable of entries like (gx,gy,mx,my,gap) or dicts with mx,my
    gxw, gyw = goal_world
    best = None; best_dist = float('inf')
    for d in doors:
        if isinstance(d, dict):
            mwx, mwy = d["mx"], d["my"]
            gap = d.get("gap", 0.0)
            # grid indices unknown here (caller should convert if needed)
            d_entry = (None, None, mwx, mwy, gap)
        else:
            d_entry = d
            mwx, mwy = d_entry[2], d_entry[3]
        ddist = math.hypot(mwx - gxw, mwy - gyw)
        if ddist < best_dist:
            best_dist = ddist
            best = d_entry
    return best

# ---------------- Main ----------------
def main():
    global last_detection_time, door_history

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    user_next = threading.Event()

    load_history()

    print("CSV mode enabled. Loading scans from:", CSV_FOLDER)
    worker = threading.Thread(target=csv_worker_loop, args=(CSV_FOLDER, user_next), daemon=True)
    worker.start()

    plt.ion(); fig, ax = plt.subplots(figsize=(8,8)); plt.show(block=False)

    visited_map = None   # will be initialized when we have map size

    def on_key(event):
        if event.key == 'q':
            plt.close(); sys.exit(0)
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

        # initialize visited_map the first time (same shape as occupancy)
        if visited_map is None:
            visited_map = np.zeros_like(occ_map)

        # Smart goal selection:
        current_time = time.time()
        goal_door = None

        # Rule 1: prefer fresh detections (choose one closest to ultimate goal)
        if clustered:
            goal_door = closest_door_to_goal(clustered, ULTIMATE_GOAL)

        # Rule 2: fallback to history if no fresh detections for a while
        elif current_time - last_detection_time > DETECTION_COOLDOWN and door_history:
            # door_history stores world midpoints, convert to entries like (gx,gy,mx,my,gap)
            past_entries = []
            for d in door_history:
                # convert world midpoint to grid
                try:
                    gx, gy = world_to_grid(min_x, min_y, d["mx"], d["my"], XY_RESOLUTION)
                    past_entries.append((gx, gy, d["mx"], d["my"], d.get("gap", 0.0)))
                except Exception:
                    continue
            if past_entries:
                goal_door = closest_door_to_goal(past_entries, ULTIMATE_GOAL)

        # Determine grid goal coords
        if goal_door:
            gx, gy, mx, my, gap = goal_door
            goal_grid = (gx, gy)
        else:
            # fallback to ultimate goal (world->grid)
            ugx, ugy = world_to_grid(min_x, min_y, *ULTIMATE_GOAL, XY_RESOLUTION)
            goal_grid = (ugx, ugy)

        start = (cx, cy)
        goal = goal_grid

        # Plan path with A*
        path = astar(occ_map, start, goal)

        # Mark visited cells along the path (so visited_map accumulates)
        if path:
            for (px, py) in path:
                if 0 <= px < visited_map.shape[0] and 0 <= py < visited_map.shape[1]:
                    visited_map[px, py] = 1.0

        # Prepare visualization
        ax.clear()
        ax.imshow(occ_map.T, cmap="gray_r", origin="lower")

        # overlay visited map as blue tint (alpha)
        ax.imshow(visited_map.T, cmap="Blues", origin="lower", alpha=0.25)

        # plot robot
        ax.plot(cx, cy, "og", label="Robot")

        # plot clustered doors (fresh detections) in yellow
        for (dgx, dgy, dmx, dmy, dgap) in clustered:
            ax.plot(dgx, dgy, "yo", markersize=6)
            ax.text(dgx+1, dgy+1, f"{dgap:.2f}m", fontsize=6, color='y')

        # identify unexplored doors: doors (fresh or historical) whose grid cell hasn't been visited
        unexplored_doors = []

        # check fresh clustered doors
        for (dgx, dgy, dmx, dmy, dgap) in clustered:
            if 0 <= dgx < visited_map.shape[0] and 0 <= dgy < visited_map.shape[1]:
                if visited_map[dgx, dgy] < 0.5:
                    unexplored_doors.append((dgx, dgy))

        # also check historical doors
        for d in door_history:
            try:
                hx, hy = world_to_grid(min_x, min_y, d["mx"], d["my"], XY_RESOLUTION)
                if 0 <= hx < visited_map.shape[0] and 0 <= hy < visited_map.shape[1]:
                    if visited_map[hx, hy] < 0.5:
                        unexplored_doors.append((hx, hy))
            except:
                pass

        # plot unexplored doors (red circles)
        for (ux, uy) in unexplored_doors:
            ax.plot(ux, uy, "r^", markersize=6, label="Unexplored door")

        # plot planned path
        if path:
            pxs, pys = zip(*path)
            ax.plot(pxs, pys, "b-", linewidth=1.5, label="Path")

        # plot ultimate goal as red X
        ugx, ugy = world_to_grid(min_x, min_y, *ULTIMATE_GOAL, XY_RESOLUTION)
        ax.plot(ugx, ugy, "kx", markersize=9, label="Ultimate Goal")

        ax.legend(loc='upper right')
        plt.pause(UI_PAUSE)
        time.sleep(0.05)

if __name__ == "__main__":
    main()
