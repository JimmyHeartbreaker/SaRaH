#!/usr/bin/env python3
"""
Multithreaded LiDAR -> Grid map -> Doorway detection (no PyQt)
- Main thread: Matplotlib UI & keyboard
- Worker thread: LiDAR reading, occupancy map, doorway detection + clustering
- Simulated driving / arrival marking in separate thread (replace with real motion)

Enhancements:
- ULTIMATE_GOAL: final destination the robot should head toward (user fills in)
- choose doorways that lead toward the goal (not just nearest)
- path_stack + visited + dead_ends to backtrack if dead end encountered
- optional path_history.json saving
"""

import time
import math
import signal
import subprocess
import sys
import threading
import json

from collections import deque

import numpy as np
import matplotlib.pyplot as plt

# ---------------- Configuration ----------------
EXE_PATH = r"C:\uni\Hub-main\Hub-main\Debug\ultra_simple.exe"
PORT = "COM4"
BAUD = "460800"

XY_RESOLUTION = 0.02  # meters per grid cell (you confirmed 2cm)
EXTEND_AREA = 3.0

# Doorway detection params
ROBOT_DIAMETER = 0.30  # meters
SAFETY_MARGIN = 0.05   # meters
DOOR_THRESHOLD = ROBOT_DIAMETER + SAFETY_MARGIN  # ~0.35 m threshold
MIN_VALID_RANGE = 0.05  # meters

# Clustering / anti-bounce params
CLUSTER_EPS = 0.25  # meters - cluster radius for doorway points
CLUSTER_MIN_SAMPLES = 1
MIN_DOORWAY_SEPARATION = 0.7  # meters, don't choose new door closer than this to last visited
ARRIVAL_RADIUS = 0.4  # meters - when simulated arrival is within this, mark visited
DETECTION_COOLDOWN = 12  # number of worker cycles to suppress selections after arrival

# Simulation / runtime params
SIM_DRIVE_SPEED = 0.20  # m/s when simulating travel
RING_RADIUS_M = 1
SAMPLE_COUNT = 360
WORKER_SCAN_SECONDS = 1.0  # accumulate ~1s scans per worker cycle

# UI timing
UI_PAUSE = 0.05  # seconds for plt.pause

# Where to save path history (optional)
PATH_HISTORY_FILE = "path_history.json"

# ---------------- Set your ultimate goal here (world coords in meters) ----------------
# If you want to test fallback to nearest-door behavior, set to None.
# Example: ULTIMATE_GOAL = (2.5, -1.2)
ULTIMATE_GOAL = (1.5, 1.5)

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
    """
    Build occupancy grid: 0.0 free, 1.0 occupied, 0.5 unknown
    Returns: occupancy_map, center_x, center_y, min_x, min_y, x_w, y_w
    """
    min_x, min_y, max_x, max_y, x_w, y_w = calc_grid_map_config(ox, oy, xy_resolution)
    # ensure positive sizes
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
                occupancy_map[lx][ly] = 0.0  # free along ray
        # mark endpoint as occupied (2x2)
        for dx in range(2):
            for dy in range(2):
                nx = ix + dx
                ny = iy + dy
                if 0 <= nx < x_w and 0 <= ny < y_w:
                    occupancy_map[nx][ny] = 1.0
    return occupancy_map, center_x, center_y, min_x, min_y, x_w, y_w

# ---------------- A* Planner ----------------
class AStarPlanner:
    def __init__(self, obstacle_map):
        self.obstacle_map = obstacle_map
        self.x_width, self.y_width = obstacle_map.shape
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(sx, sy, 0.0, -1)
        goal_node = self.Node(gx, gy, 0.0, -1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node

        while True:
            if not open_set:
                return [], []
            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]
            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
            del open_set[c_id]
            closed_set[c_id] = current
            for move in self.motion:
                node = self.Node(current.x + move[0], current.y + move[1], current.cost + move[2], c_id)
                n_id = self.calc_index(node)
                if not self.verify_node(node):
                    continue
                if n_id in closed_set:
                    continue
                if n_id not in open_set or open_set[n_id].cost > node.cost:
                    open_set[n_id] = node

        return self.calc_final_path(goal_node, closed_set)

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [goal_node.x], [goal_node.y]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index
        return rx, ry

    def calc_index(self, node):
        return node.y * self.x_width + node.x

    def calc_heuristic(self, n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

    def verify_node(self, node):
        if node.x < 0 or node.y < 0 or node.x >= self.x_width or node.y >= self.y_width:
            return False
        if self.obstacle_map[node.x][node.y] >= 0.9:
            return False
        return True

    @staticmethod
    def get_motion_model():
        return [
            [1,0,1], [0,1,1], [-1,0,1], [0,-1,1],
            [-1,-1,math.sqrt(2)], [-1,1,math.sqrt(2)],
            [1,-1,math.sqrt(2)], [1,1,math.sqrt(2)]
        ]

# ---------------- Doorway detection (grid ring sampling) ----------------
def grid_to_world(min_x, min_y, ix, iy, xy_resolution):
    x = min_x + ix * xy_resolution
    y = min_y + iy * xy_resolution
    return x, y

def detect_doorways_in_grid(occupancy_map, center_x, center_y, min_x, min_y, xy_resolution,
                            radius_m=RING_RADIUS_M, sample_count=SAMPLE_COUNT, door_threshold=DOOR_THRESHOLD):
    """
    Detect door candidates by sampling a ring around the robot.
    Returns raw candidates as tuples: (mx_world, my_world, gap_m, gx_idx, gy_idx)
    where gx_idx,gy_idx is a representative grid index (midpoint sample).
    """
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
            samples.append((ang, None, None, 1.0))  # treat out-of-bounds as occupied

    # find continuous free segments
    segments = []
    current = []
    for ang, gx, gy, val in samples + [samples[0]]:  # wrap-around
        if gx is not None and val < 0.4:
            current.append((ang, gx, gy))
        else:
            if current:
                segments.append(current)
                current = []
            else:
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

# ---------------- Greedy clustering (no sklearn) ----------------
def cluster_points(points, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES):
    """
    points: list of (x,y) in meters
    returns: list of cluster centers (x,y) in meters
    Simple greedy clustering: group points within eps of existing cluster centers.
    """
    clusters = []
    members = []
    for p in points:
        added = False
        for i, c in enumerate(clusters):
            if math.hypot(p[0] - c[0], p[1] - c[1]) <= eps:
                members[i].append(p)
                xs = [q[0] for q in members[i]]; ys = [q[1] for q in members[i]]
                clusters[i] = (sum(xs)/len(xs), sum(ys)/len(ys))
                added = True
                break
        if not added:
            clusters.append((p[0], p[1]))
            members.append([p])
    # filter small clusters
    final = []
    for i, mem in enumerate(members):
        if len(mem) >= min_samples:
            xs = [q[0] for q in mem]; ys = [q[1] for q in mem]
            final.append((sum(xs)/len(xs), sum(ys)/len(ys)))
    return final

# ---------------- Shared state & synchronization ----------------
shared = {
    "occupancy_map": None,
    "center_x": None,
    "center_y": None,
    "min_x": None,
    "min_y": None,
    "x_w": None,
    "y_w": None,
    "raw_candidates": [],   # raw door candidates (mwx,mwy,gap,gx,gy)
    "clustered_doors": [],  # clustered, filtered doors (gx,gy,mx,my,gap)
    "chosen_path": None,    # (rx,ry,gx,gy,wx,wy,gap)
    "waiting_at_doorway": False,
    "continue_search": True,
    "cooldown": 0
}
state_lock = threading.Lock()
visited_doors = set()  # set of (rounded_world_x, rounded_world_y)
last_goal_world = None

# Navigation memory for backtracking
visited_goals = set()            # same keys as used below (rounded world)
visited_details = dict()         # key -> (gx, gy, mx, my, gap)
path_stack = []                  # stack of keys (order of traversal); top is current doorway
dead_ends = set()                # keys marked as dead ends
path_history = []                # append visited keys for saving optionally

# ---------------- Start LIDAR process ----------------
def start_lidar_proc():
    try:
        cmd = [EXE_PATH, "--channel", "--serial", PORT, BAUD]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        return proc
    except Exception as e:
        print("Warning: could not start lidar process, running in offline/sim mode.", e)
        return None

# ---------------- Background worker (reads lidar, creates map, detects doors) ----------------
def lidar_worker(proc_handle):
    """
    Worker loop:
     - read ~WORKER_SCAN_SECONDS of lidar lines
     - build occupancy map
     - detect raw door candidates
     - cluster raw candidates into door centers
     - filter by visited / cooldown
     - write into shared state
    """
    global last_goal_world
    while True:
        angles = []
        distances = []
        t0 = time.time()
        # gather ~1 second of lidar frames
        while time.time() - t0 < WORKER_SCAN_SECONDS:
            if proc_handle is None:
                # offline mode: nothing to read, break to avoid busy loop
                time.sleep(0.05)
                break
            line = proc_handle.stdout.readline()
            if not line:
                continue
            parts = line.strip().split(",")
            if len(parts) < 2:
                continue
            try:
                angle_deg = float(parts[0]); dist_mm = float(parts[1])
                if dist_mm <= 0: continue
                angles.append(math.radians(angle_deg))
                distances.append(dist_mm / 1000.0)
            except ValueError:
                continue

        if not angles:
            # decay cooldown even if no scan
            with state_lock:
                if shared["cooldown"] > 0:
                    shared["cooldown"] -= 1
            time.sleep(0.01)
            continue

        angs = np.array(angles)
        dists = np.array(distances)
        ox = np.sin(angs) * dists
        oy = np.cos(angs) * dists

        occ_map, cx, cy, min_x, min_y, x_w, y_w = generate_ray_casting_grid_map(ox, oy, XY_RESOLUTION)

        raw_candidates = detect_doorways_in_grid(occ_map, cx, cy, min_x, min_y, XY_RESOLUTION)

        # filter out already visited (rounded world coords)
        raw_world = []
        for (mwx, mwy, gap, gx, gy) in raw_candidates:
            key = (round(mwx, 2), round(mwy, 2))
            if key in visited_doors:
                continue
            raw_world.append((mwx, mwy, gap, gx, gy))

        # cluster world points (mwx,mwy)
        pts = [(p[0], p[1]) for p in raw_world]
        clusters = cluster_points(pts, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES)

        # for each cluster, find nearest raw candidate to obtain grid coords and gap
        clustered = []
        for (cxw, cyw) in clusters:
            best = None
            best_dist = float('inf')
            for (mwx, mwy, gap, gx, gy) in raw_world:
                d = math.hypot(cxw - mwx, cyw - mwy)
                if d < best_dist:
                    best_dist = d
                    best = (gx, gy, mwx, mwy, gap)
            if best:
                clustered.append(best)  # (gx,gy,mx,my,gap)

        # filter by distance from last visited goal and apply cooldown semantics
        filtered = []
        with state_lock:
            cooldown = shared["cooldown"]
            lg = last_goal_world
        for (gx, gy, mx, my, gap) in clustered:
            if cooldown > 0:
                # still include but selection will be suppressed by main thread until cooldown ends
                filtered.append((gx, gy, mx, my, gap))
                continue
            if last_goal_world is not None:
                if math.hypot(mx - last_goal_world[0], my - last_goal_world[1]) < MIN_DOORWAY_SEPARATION:
                    continue
            filtered.append((gx, gy, mx, my, gap))

        with state_lock:
            shared["occupancy_map"] = occ_map
            shared["center_x"] = cx
            shared["center_y"] = cy
            shared["min_x"] = min_x
            shared["min_y"] = min_y
            shared["x_w"] = x_w
            shared["y_w"] = y_w
            shared["raw_candidates"] = raw_world
            shared["clustered_doors"] = filtered

        # tiny sleep to yield
        time.sleep(0.01)

# ---------------- Utility helpers ----------------
def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def choose_doorway_by_goal(clustered_doors, goal):
    """
    clustered_doors: list of (gx,gy,mx,my,gap)
    goal: (x,y) world coords or None
    Returns best doorway tuple (gx,gy,mx,my,gap) or None
    """
    if not clustered_doors:
        return None

    # Build key -> tuple for convenience
    choices = []
    for (gx, gy, mx, my, gap) in clustered_doors:
        key = (round(mx, 2), round(my, 2))
        choices.append((key, (gx, gy, mx, my, gap)))

    # Exclude dead_ends and visited - we check visited externally; this function focuses on goal
    if goal is None:
        # fallback to nearest to robot (mx,my distance)
        choices_sorted = sorted(choices, key=lambda kv: math.hypot(kv[1][2], kv[1][3]))
        return choices_sorted[0][1]
    else:
        # choose doorway whose world position is closest to goal
        choices_sorted = sorted(choices, key=lambda kv: euclidean((kv[1][2], kv[1][3]), goal))
        return choices_sorted[0][1]

def save_path_history(filename=PATH_HISTORY_FILE):
    try:
        with open(filename, "w") as f:
            json.dump(path_history, f, indent=2)
    except Exception as e:
        print("Failed to save path history:", e)

# ---------------- Main (Matplotlib UI in main thread) ----------------

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    proc = start_lidar_proc()

    worker = threading.Thread(target=lidar_worker, args=(proc,), daemon=True)
    worker.start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(7,7))
    plt.show(block=False)

    # --- Control flag ---
    user_next = threading.Event()

    def on_key(event):
        if event.key == 'q':
            print("Quit requested (q).")
            try:
                if proc:
                    proc.terminate()
            except Exception:
                pass
            plt.close()
            # Save path history on exit
            save_path_history()
            sys.exit(0)
        elif event.key == 'n':
            print("Continuing to next scan... (user pressed 'n')")
            user_next.set()
        elif event.key == 's':
            print("Saving path history now...")
            save_path_history()

    fig.canvas.mpl_connect('key_press_event', on_key)

    print("Starting doorway exploration.")
    print("Controls: 'n' = mark arrival / continue; 'q' = quit; 's' = save path history\n")

    try:
        # main loop
        while True:
            # --- Wait for new occupancy map from LiDAR ---
            print("Waiting for updated map...")
            while True:
                with state_lock:
                    occ_map = shared["occupancy_map"]
                    cx = shared["center_x"]
                    cy = shared["center_y"]
                    clustered = list(shared["clustered_doors"])
                if occ_map is not None:
                    break
                time.sleep(0.2)

            # --- Plot map ---
            ax.clear()
            ax.imshow(occ_map.T, cmap="gray_r", origin="lower")
            ax.plot(cx, cy, "og", label="LIDAR")
            for (gx, gy, mx, my, gap) in clustered:
                ax.plot(gx, gy, "yo", markersize=7)
                ax.text(gx + 1, gy + 1, f"{gap:.2f}m", fontsize=6, color='y')
            # highlight path stack on the map (approx by plotting stored gx,gy)
            for i, key in enumerate(path_stack):
                detail = visited_details.get(key)
                if detail:
                    gx, gy = detail[0], detail[1]
                    ax.plot(gx, gy, "bx", markersize=6)  # previously traversed
            plt.pause(0.1)

            # --- If there are no doors, just wait for next map ---
            if not clustered:
                print("No doorways found. Waiting for next scan...")
                ax.set_title("No doorways — waiting for new map.")
                plt.pause(0.5)

                # Wait for user to trigger new scan
                user_next.clear()
                while not user_next.is_set():
                    plt.pause(0.1)

                # Clear old map to force lidar refresh
                with state_lock:
                    shared["occupancy_map"] = None
                    shared["clustered_doors"] = []
                continue

            # --- Filter clustered doors against visited / dead_ends ---
            clustered_filtered = []
            for (gx, gy, mx, my, gap) in clustered:
                key = (round(mx, 2), round(my, 2))
                if key in dead_ends:
                    continue
                clustered_filtered.append((gx, gy, mx, my, gap))

            # --- Choose doorway by goal preference ---
            candidate = choose_doorway_by_goal(clustered_filtered, ULTIMATE_GOAL)
            next_goal = None

            # If candidate already visited, search for next unvisited
            if candidate:
                # generate sorted list of doors by goal closeness (so we can pick next if top is visited)
                def sort_key(d):
                    if ULTIMATE_GOAL is None:
                        return math.hypot(d[2], d[3])  # distance to robot
                    return euclidean((d[2], d[3]), ULTIMATE_GOAL)
                doors_sorted = sorted(clustered_filtered, key=sort_key)
                for (gx, gy, mx, my, gap) in doors_sorted:
                    key = (round(mx, 2), round(my, 2))
                    if key in visited_goals or key in dead_ends:
                        continue
                    # also avoid trivially close doors (optional)
                    next_goal = (gx, gy, mx, my, gap)
                    break

            if next_goal is None:
                # No unvisited doors available in current scan -> backtrack if possible
                if path_stack:
                    print("No new doorways available. Backtracking to previous waypoint...")
                    # pop current (we are presumably at top); then target previous
                    current_key = path_stack.pop() if path_stack else None
                    if current_key:
                        # mark the current as dead end if nothing left after it
                        dead_ends.add(current_key)
                    if not path_stack:
                        print("Backtracked all the way to start; nothing left to try.")
                        ax.set_title("Explored all reachable doors.")
                        plt.pause(0.5)
                        # wait for user to press 'n' to continue rescanning or quit
                        user_next.clear()
                        while not user_next.is_set():
                            plt.pause(0.1)
                        with state_lock:
                            shared["occupancy_map"] = None
                            shared["clustered_doors"] = []
                        continue
                    prev_key = path_stack[-1]
                    prev_detail = visited_details.get(prev_key)
                    if prev_detail:
                        gx, gy, mx, my, gap = prev_detail
                        print(f"Backtracking to {prev_key} at grid ({gx},{gy})")
                        # plan path to prev grid cell
                        a_star = AStarPlanner(occ_map)
                        rx, ry = a_star.planning(cx, cy, gx, gy)
                        if not rx:
                            print("Backtrack path planning failed; skipping this backtrack node.")
                            # mark as dead end and continue loop to pop next time
                            dead_ends.add(prev_key)
                            continue
                        ax.plot(rx, ry, "-r", label=f"Backtrack to {prev_key}")
                        ax.plot(gx, gy, "yx", markersize=10)
                        ax.legend(loc='upper right')
                        ax.set_title(f"Backtracking to {prev_key}")
                        plt.pause(0.5)
                        # simulate arrival: wait for 'n'
                        print("Arrived at previous waypoint. Press 'n' to continue scanning.")
                        user_next.clear()
                        while not user_next.is_set():
                            plt.pause(0.1)
                        with state_lock:
                            shared["occupancy_map"] = None
                            shared["clustered_doors"] = []
                        continue
                    else:
                        # no detail recorded for previous (rare) - pop and continue
                        path_stack.pop()
                        continue
                else:
                    # nothing to backtrack to
                    print("No unvisited doorways and no path history to backtrack. Waiting for user input.")
                    ax.set_title("No doorways and no history — waiting.")
                    plt.pause(0.5)
                    user_next.clear()
                    while not user_next.is_set():
                        plt.pause(0.1)
                    with state_lock:
                        shared["occupancy_map"] = None
                        shared["clustered_doors"] = []
                    continue

            # --- Plan path to the selected next doorway ---
            gx, gy, mx, my, gap = next_goal
            key = (round(mx, 2), round(my, 2))
            print(f"\nPlanning path to doorway {key} (gap {gap:.2f} m) aiming for goal {ULTIMATE_GOAL}")

            a_star = AStarPlanner(occ_map)
            rx, ry = a_star.planning(cx, cy, gx, gy)

            if not rx:
                print("Path planning failed. Marking doorway as dead end and continuing.")
                dead_ends.add(key)
                visited_goals.add(key)
                visited_details[key] = (gx, gy, mx, my, gap)
                path_history.append({"action": "mark_dead_end", "door": key})
                save_path_history()
                continue

            # --- Plot path ---
            ax.plot(rx, ry, "-r", label=f"Path to {key}")
            ax.plot(gx, gy, "yx", markersize=10)
            ax.legend(loc='upper right')
            ax.set_title(f"Travelling to {key}")
            plt.pause(0.5)

            # --- Simulate reaching doorway ---
            print(f"Reached doorway {key}. Waiting for 'n' to update map (press 'n' when you want to continue)...")
            # record visit and push to path stack
            visited_goals.add(key)
            visited_details[key] = (gx, gy, mx, my, gap)
            path_stack.append(key)
            path_history.append({"action": "visit", "door": key, "world": (mx, my), "gap": gap})
            save_path_history()

            user_next.clear()
            while not user_next.is_set():
                plt.pause(0.1)

            # After user confirms 'n' (i.e., map should be updated), we clear the map and wait for next LiDAR cycle
            with state_lock:
                shared["occupancy_map"] = None
                shared["clustered_doors"] = []
            print("Map cleared. Waiting for next LiDAR update...\n")

    except KeyboardInterrupt:
        try:
            if proc:
                proc.terminate()
        except Exception:
            pass
        save_path_history()
        plt.close()


if __name__ == "__main__":
    main()
