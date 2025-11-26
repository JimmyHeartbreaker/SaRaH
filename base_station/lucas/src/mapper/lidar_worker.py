# lidar_worker.py
import time, math, subprocess, threading
import numpy as np
from gridmap import generate_ray_casting_grid_map
from doorway_detection import detect_doorways_in_grid, cluster_points
from config import *

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
    "cooldown": 0
}
state_lock = threading.Lock()
visited_doors = set()
last_goal_world = None


def start_lidar_proc():
    try:
        cmd = [EXE_PATH, "--channel", "--serial", PORT, BAUD]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        return proc
    except Exception as e:
        print("Warning: could not start lidar process:", e)
        return None


def lidar_worker(proc_handle):
    global last_goal_world
    while True:
        angles, distances = [], []
        t0 = time.time()
        while time.time() - t0 < WORKER_SCAN_SECONDS:
            if proc_handle is None:
                time.sleep(0.05)
                break
            line = proc_handle.stdout.readline()
            if not line:
                continue
            parts = line.strip().split(",")
            if len(parts) < 2:
                continue
            try:
                angle_deg = float(parts[0])
                dist_mm = float(parts[1])
                if dist_mm <= 0:
                    continue
                angles.append(math.radians(angle_deg))
                distances.append(dist_mm / 1000.0)
            except ValueError:
                continue

        if not angles:
            with state_lock:
                if shared["cooldown"] > 0:
                    shared["cooldown"] -= 1
            time.sleep(0.05)
            continue

        angs = np.array(angles)
        dists = np.array(distances)
        ox = np.sin(angs) * dists
        oy = np.cos(angs) * dists

        occ_map, cx, cy, min_x, min_y, x_w, y_w = generate_ray_casting_grid_map(ox, oy, XY_RESOLUTION)
        raw_candidates = detect_doorways_in_grid(occ_map, cx, cy, min_x, min_y, XY_RESOLUTION)

        raw_world = []
        for (mwx, mwy, gap, gx, gy) in raw_candidates:
            key = (round(mwx, 2), round(mwy, 2))
            if key in visited_doors:
                continue
            raw_world.append((mwx, mwy, gap, gx, gy))

        pts = [(p[0], p[1]) for p in raw_world]
        clusters = cluster_points(pts, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES)

        clustered = []
        for (cxw, cyw) in clusters:
            best = None
            best_dist = float("inf")
            for (mwx, mwy, gap, gx, gy) in raw_world:
                d = math.hypot(cxw - mwx, cyw - mwy)
                if d < best_dist:
                    best_dist = d
                    best = (gx, gy, mwx, mwy, gap)
            if best:
                clustered.append(best)

        with state_lock:
            shared.update({
                "occupancy_map": occ_map,
                "center_x": cx,
                "center_y": cy,
                "min_x": min_x,
                "min_y": min_y,
                "x_w": x_w,
                "y_w": y_w,
                "raw_candidates": raw_world,
                "clustered_doors": clustered
            })

        time.sleep(0.01)
