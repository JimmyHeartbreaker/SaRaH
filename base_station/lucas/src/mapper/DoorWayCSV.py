#!/usr/bin/env python3
"""
Offline LiDAR -> Grid map -> Doorway detection (CSV input)
Simulates the full exploration / path stack / backtracking logic
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
CLUSTER_EPS = 0.25
CLUSTER_MIN_SAMPLES = 1
MIN_DOORWAY_SEPARATION = 0.7
PATH_HISTORY_FILE = "path_history.json"

# ---------------- Set ultimate goal ----------------
ULTIMATE_GOAL = (1.5, 1.5)  # or None

# ---------------- Bresenham ----------------
def bresenham(start, end):
    x1, y1 = start; x2, y2 = end
    dx = x2 - x1; dy = y2 - y1
    is_steep = abs(dy) > abs(dx)
    if is_steep: x1, y1, x2, y2 = y1, x1, y2, x2
    swapped = False
    if x1 > x2: x1, x2, y1, y2, swapped = x2, x1, y2, y1, True
    dx = x2 - x1; dy = y2 - y1
    error = int(dx / 2.0); y_step = 1 if y1 < y2 else -1; y = y1
    points = []
    for x in range(x1, x2 + 1):
        points.append([y, x] if is_steep else (x, y))
        error -= abs(dy)
        if error < 0: y += y_step; error += dx
    if swapped: points.reverse()
    return np.array(points)

# ---------------- Grid map helpers ----------------
EXTEND_AREA = 3.0
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
    x_w = max(1, x_w); y_w = max(1, y_w)
    occupancy_map = np.ones((x_w, y_w)) / 2.0
    center_x = int(round(-min_x / xy_resolution))
    center_y = int(round(-min_y / xy_resolution))
    for (x, y) in zip(ox, oy):
        ix = int(round((x - min_x) / xy_resolution))
        iy = int(round((y - min_y) / xy_resolution))
        if ix < 0 or iy < 0 or ix >= x_w or iy >= y_w: continue
        laser_beams = bresenham((center_x, center_y), (ix, iy))
        for lx, ly in laser_beams:
            if 0 <= lx < x_w and 0 <= ly < y_w: occupancy_map[lx][ly] = 0.0
        for dx in range(2):
            for dy in range(2):
                nx, ny = ix+dx, iy+dy
                if 0 <= nx < x_w and 0 <= ny < y_w: occupancy_map[nx][ny] = 1.0
    return occupancy_map, center_x, center_y, min_x, min_y, x_w, y_w

# ---------------- Doorway detection ----------------
def grid_to_world(min_x, min_y, ix, iy, xy_resolution):
    return min_x + ix*xy_resolution, min_y + iy*xy_resolution

def detect_doorways_in_grid(occupancy_map, center_x, center_y, min_x, min_y, xy_resolution,
                            radius_m=1.0, sample_count=360, door_threshold=DOOR_THRESHOLD):
    doorways = []
    x_w, y_w = occupancy_map.shape
    radius_cells = max(1, int(round(radius_m / xy_resolution)))
    angles = np.linspace(0, 2*math.pi, sample_count, endpoint=False)
    samples = []
    for ang in angles:
        gx = int(round(center_x + radius_cells * math.cos(ang)))
        gy = int(round(center_y + radius_cells * math.sin(ang)))
        val = occupancy_map[gx, gy] if 0 <= gx < x_w and 0 <= gy < y_w else 1.0
        samples.append((ang, gx, gy, val if 0 <= gx < x_w and 0 <= gy < y_w else 1.0))
    segments, current = [], []
    for ang, gx, gy, val in samples + [samples[0]]:
        if gx is not None and val < 0.4: current.append((ang, gx, gy))
        else:
            if current: segments.append(current); current=[]
    for seg in segments:
        if len(seg) < 2: continue
        _, ix1, iy1 = seg[0]; _, ix2, iy2 = seg[-1]
        wx1, wy1 = grid_to_world(min_x, min_y, ix1, iy1, xy_resolution)
        wx2, wy2 = grid_to_world(min_x, min_y, ix2, iy2, xy_resolution)
        gap = math.hypot(wx1-wx2, wy1-wy2)
        if gap >= door_threshold:
            mid_idx = len(seg)//2
            _, mxg, myg = seg[mid_idx]
            mwx, mwy = grid_to_world(min_x, min_y, mxg, myg, xy_resolution)
            doorways.append((mwx, mwy, gap, mxg, myg))
    return doorways

# ---------------- Clustering ----------------
def cluster_points(points, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES):
    clusters, members = [], []
    for p in points:
        added=False
        for i,c in enumerate(clusters):
            if math.hypot(p[0]-c[0], p[1]-c[1])<=eps:
                members[i].append(p)
                xs=[q[0] for q in members[i]]; ys=[q[1] for q in members[i]]
                clusters[i]=(sum(xs)/len(xs), sum(ys)/len(ys))
                added=True; break
        if not added: clusters.append((p[0],p[1])); members.append([p])
    final=[]
    for i, mem in enumerate(members):
        if len(mem)>=min_samples:
            xs=[q[0] for q in mem]; ys=[q[1] for q in mem]
            final.append((sum(xs)/len(xs), sum(ys)/len(ys)))
    return final

# ---------------- Shared state ----------------
shared = {"occupancy_map":None,"center_x":None,"center_y":None,"min_x":None,"min_y":None,
          "x_w":None,"y_w":None,"raw_candidates":[],"clustered_doors":[]}
state_lock = threading.Lock()

# ---------------- Path / visit memory ----------------
visited_goals = set()
visited_details = dict()
path_stack = []
dead_ends = set()
path_history = []

def euclidean(a,b): return math.hypot(a[0]-b[0],a[1]-b[1])

def choose_doorway_by_goal(clustered_doors, goal):
    if not clustered_doors: return None
    if goal is None:
        return min(clustered_doors, key=lambda d: math.hypot(d[2],d[3]))
    return min(clustered_doors, key=lambda d: euclidean((d[2],d[3]), goal))

def save_path_history(filename=PATH_HISTORY_FILE):
    try: json.dump(path_history, open(filename,'w'), indent=2)
    except Exception as e: print("Failed to save path history:", e)

# ---------------- CSV worker ----------------
def csv_worker(csv_file):
    data = np.loadtxt(csv_file, delimiter=',', skiprows=1)  # skip header row
    ox, oy = data[:,0], data[:,1]
    occ_map, cx, cy, min_x, min_y, x_w, y_w = generate_ray_casting_grid_map(ox, oy, XY_RESOLUTION)
    raw_candidates = detect_doorways_in_grid(occ_map, cx, cy, min_x, min_y, XY_RESOLUTION)
    pts = [(p[0],p[1]) for p in raw_candidates]
    clusters = cluster_points(pts)
    clustered=[]
    for cxw,cyw in clusters:
        best=min(raw_candidates,key=lambda r: math.hypot(r[0]-cxw,r[1]-cyw))
        clustered.append(best)
    with state_lock:
        shared.update({"occupancy_map":occ_map,"center_x":cx,"center_y":cy,
                       "min_x":min_x,"min_y":min_y,"x_w":x_w,"y_w":y_w,
                       "raw_candidates":raw_candidates,"clustered_doors":clustered})

# ---------------- Main loop ----------------
def main(csv_file):
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    worker=threading.Thread(target=csv_worker,args=(csv_file,),daemon=True)
    worker.start()

    plt.ion(); fig, ax = plt.subplots(figsize=(7,7)); plt.show(block=False)
    user_next = threading.Event()

    def on_key(event):
        if event.key=='q': plt.close(); save_path_history(); sys.exit(0)
        elif event.key=='n': user_next.set()
        elif event.key=='s': save_path_history()

    fig.canvas.mpl_connect('key_press_event',on_key)

    print("Offline doorway exploration. Press 'n' to visit next doorway, 'q' to quit, 's' to save.")

    while True:
        with state_lock:
            occ_map = shared["occupancy_map"]
            clustered = list(shared["clustered_doors"])
            cx = shared["center_x"]
            cy = shared["center_y"]

        if occ_map is None:
            time.sleep(0.1)
            continue

        ax.clear(); ax.imshow(occ_map.T,cmap="gray_r",origin="lower"); ax.plot(cx,cy,"og")
        for mwx,mwy,gap,gx,gy in clustered: ax.plot(gx,gy,"yo")
        for key in path_stack:
            detail = visited_details.get(key)
            if detail: ax.plot(detail[0],detail[1],"bx")
        plt.pause(0.1)

        # Filter unvisited doors
        doors_filtered=[d for d in clustered if (round(d[2],2),round(d[3],2)) not in visited_goals and (round(d[2],2),round(d[3],2)) not in dead_ends]
        if not doors_filtered:
            print("No unvisited doors left. Waiting for 'n' to reset or quit.")
            user_next.clear()
            while not user_next.is_set(): plt.pause(0.1)
            shared["occupancy_map"]=None; shared["clustered_doors"]=[]
            continue

        next_goal = choose_doorway_by_goal(doors_filtered, ULTIMATE_GOAL)
        if next_goal is None: continue

        gx,gy,mx,my,gap = next_goal
        key=(round(mx,2),round(my,2))
        print(f"Visiting doorway {key} gap={gap:.2f}m")

        visited_goals.add(key)
        visited_details[key]=(gx,gy,mx,my,gap)
        path_stack.append(key)
        path_history.append({"action":"visit","door":key,"world":(mx,my),"gap":gap})
        save_path_history()

        user_next.clear()
        while not user_next.is_set(): plt.pause(0.1)
        shared["occupancy_map"]=None; shared["clustered_doors"]=[]
        print("Ready for next doorway scan.\n")

if __name__=="__main__":
    if len(sys.argv)<2: print("Usage: python script.py points.csv"); sys.exit(1)
    main(sys.argv[1])
