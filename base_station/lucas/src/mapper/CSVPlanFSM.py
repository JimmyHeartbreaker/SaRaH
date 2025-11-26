#!/usr/bin/env python3
"""
FSM-style CSV-based LiDAR simulation with doorway detection and path planning.
Controls:
  - 'n' : load next CSV scan and process
  - 'q' : quit
"""

import os
import sys
import math
import time
import csv
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt

# ---------------- Configuration ----------------
XY_RESOLUTION = 0.02  # meters per grid cell
ROBOT_DIAMETER = 0.15
SAFETY_MARGIN = 0.05
DOOR_THRESHOLD = ROBOT_DIAMETER + SAFETY_MARGIN

CLUSTER_EPS = 0.25
CLUSTER_MIN_SAMPLES = 1

RING_RADIUS_M = 0.5
SAMPLE_COUNT = 360
UI_PAUSE = 0.05

CSV_FOLDER = "scans"
ULTIMATE_GOAL = ((-5.267200469970703 / 1000.0), (451.8113708496094 / 1000.0))

# ---------------- Utilities ----------------
def bresenham(start: Tuple[int,int], end: Tuple[int,int]) -> np.ndarray:
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
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:
        points.reverse()
    return np.array(points, dtype=int)

def calc_grid_map_config(xy_resolution: float):
    # Fixed bounds (10m x 10m) centered at origin, like original code.
    min_x, max_x = -5.0, 5.0
    min_y, max_y = -5.0, 5.0
    xw = int(round((max_x - min_x) / xy_resolution))
    yw = int(round((max_y - min_y) / xy_resolution))
    return min_x, min_y, max_x, max_y, xw, yw

def generate_ray_casting_grid_map(ox: np.ndarray, oy: np.ndarray, xy_resolution: float):
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
        # mark free along the beam
        laser_beams = bresenham((center_x, center_y), (ix, iy))
        for lx, ly in laser_beams:
            if 0 <= lx < x_w and 0 <= ly < y_w:
                occupancy_map[lx, ly] = 0.0
        # mark occupied around the endpoint (simple 2x2 block)
        for dx in range(2):
            for dy in range(2):
                nx = ix + dx
                ny = iy + dy
                if 0 <= nx < x_w and 0 <= ny < y_w:
                    occupancy_map[nx, ny] = 1.0
    return occupancy_map, center_x, center_y, min_x, min_y, x_w, y_w

def grid_to_world(min_x: float, min_y: float, ix: int, iy: int, xy_resolution: float):
    x = min_x + ix * xy_resolution
    y = min_y + iy * xy_resolution
    return x, y

def world_to_grid(min_x: float, min_y: float, x: float, y: float, xy_resolution: float):
    ix = int(round((x - min_x) / xy_resolution))
    iy = int(round((y - min_y) / xy_resolution))
    return ix, iy

def detect_doorways_in_grid(occupancy_map: np.ndarray, center_x: int, center_y: int,
                            min_x: float, min_y: float, xy_resolution: float,
                            radius_m: float = RING_RADIUS_M, sample_count: int = SAMPLE_COUNT,
                            door_threshold: float = DOOR_THRESHOLD):
    doorways = []
    x_w, y_w = occupancy_map.shape
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
    # find contiguous free segments
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

def cluster_points(points: List[Tuple[float,float]], eps: float = CLUSTER_EPS, min_samples: int = CLUSTER_MIN_SAMPLES):
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

def bfs_path(occ_map: np.ndarray, start: Tuple[int,int], goal: Tuple[int,int]) -> List[Tuple[int,int]]:
    max_x, max_y = occ_map.shape
    queue = deque([start])
    came_from = {start: None}
    while queue:
        cx, cy = queue.popleft()
        if (cx, cy) == goal:
            break
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < max_x and 0 <= ny < max_y and occ_map[nx, ny] < 0.5 and (nx, ny) not in came_from:
                came_from[(nx, ny)] = (cx, cy)
                queue.append((nx, ny))
    if goal not in came_from:
        return []
    # reconstruct
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path

# ---------------- FSM ----------------
class State(Enum):
    INIT = auto()
    WAIT_FOR_INPUT = auto()
    LOAD_SCAN = auto()
    PROCESS_SCAN = auto()
    DETECT_DOORS = auto()
    SELECT_GOAL = auto()
    PLAN_PATH = auto()
    DISPLAY = auto()
    EXIT = auto()

@dataclass
class Context:
    scan_index: int = 1
    csv_folder: str = CSV_FOLDER
    occupancy_map: Optional[np.ndarray] = None
    center_x: Optional[int] = None
    center_y: Optional[int] = None
    min_x: Optional[float] = None
    min_y: Optional[float] = None
    raw_candidates: List[Tuple[float,float,float,int,int]] = None
    clustered_doors: List[Tuple[int,int,float]] = None  # list of (gx,gy,gap) in grid coords maybe
    path: List[Tuple[int,int]] = None
    ultimate_goal_grid: Optional[Tuple[int,int]] = None

class FSM:
    def __init__(self, ctx: Context):
        self.ctx = ctx
        self.state = State.INIT
        # matplotlib figure
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7,7))
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.should_advance = False
        self.should_quit = False

    def on_key(self, event):
        if event.key == 'n':
            self.should_advance = True
        elif event.key == 'q':
            self.should_quit = True

    def run(self):
        while self.state != State.EXIT:
            if self.should_quit:
                self.state = State.EXIT
            elif self.state == State.INIT:
                self.handle_init()
            elif self.state == State.WAIT_FOR_INPUT:
                self.handle_wait_for_input()
            elif self.state == State.LOAD_SCAN:
                self.handle_load_scan()
            elif self.state == State.PROCESS_SCAN:
                self.handle_process_scan()
            elif self.state == State.DETECT_DOORS:
                self.handle_detect_doors()
            elif self.state == State.SELECT_GOAL:
                self.handle_select_goal()
            elif self.state == State.PLAN_PATH:
                self.handle_plan_path()
            elif self.state == State.DISPLAY:
                self.handle_display()
            else:
                self.state = State.EXIT
            # tiny sleep to keep UI responsive
            time.sleep(0.01)
        plt.close(self.fig)
        print("Exiting FSM.")

    # ---------- Handlers ----------
    def handle_init(self):
        print("FSM Init. Press 'n' to load the first scan, 'q' to quit.")
        min_x, min_y, max_x, max_y, xw, yw = calc_grid_map_config(XY_RESOLUTION)
        ugx, ugy = world_to_grid(min_x, min_y, *ULTIMATE_GOAL, XY_RESOLUTION)
        self.ctx.ultimate_goal_grid = (ugx, ugy)
        self.state = State.WAIT_FOR_INPUT

    def handle_wait_for_input(self):
        if self.should_advance:
            self.should_advance = False
            self.state = State.LOAD_SCAN

    def handle_load_scan(self):
        path = os.path.join(self.ctx.csv_folder, f"scan_{self.ctx.scan_index}.csv")
        if not os.path.exists(path):
            print(f"No scan file: {path}")
            # stay in WAIT_FOR_INPUT allowing user to add files or quit
            self.state = State.WAIT_FOR_INPUT
            return
        # load points
        points = []
        with open(path, newline='') as f:
            reader = csv.reader(f)
            header = next(reader, None)
            for r in reader:
                if len(r) < 2:
                    continue
                try:
                    x = float(r[0]) / 1000.0
                    y = float(r[1]) / 1000.0
                    points.append((x, y))
                except ValueError:
                    continue
        if not points:
            print(f"No valid points in {path}")
            self.state = State.WAIT_FOR_INPUT
            return
        ox = np.array([p[0] for p in points])
        oy = np.array([p[1] for p in points])
        occ_map, cx, cy, min_x, min_y, x_w, y_w = generate_ray_casting_grid_map(ox, oy, XY_RESOLUTION)
        self.ctx.occupancy_map = occ_map
        self.ctx.center_x = cx
        self.ctx.center_y = cy
        self.ctx.min_x = min_x
        self.ctx.min_y = min_y
        print(f"Loaded {path} -> map size {occ_map.shape}")
        self.state = State.PROCESS_SCAN

    def handle_process_scan(self):
        # process the current occupancy grid into candidate doorways
        occ = self.ctx.occupancy_map
        if occ is None:
            self.state = State.WAIT_FOR_INPUT
            return
        raw = detect_doorways_in_grid(occ, self.ctx.center_x, self.ctx.center_y,
                                      self.ctx.min_x, self.ctx.min_y, XY_RESOLUTION)
        # cluster raw candidates
        pts = [(mwx, mwy) for (mwx, mwy, gap, gx, gy) in raw]
        clusters = cluster_points(pts, eps=CLUSTER_EPS, min_samples=CLUSTER_MIN_SAMPLES)
        clustered = []
        # find best raw candidate per cluster
        for (cxw, cyw) in clusters:
            best = None
            best_d = float('inf')
            for (mwx, mwy, gap, gx, gy) in raw:
                d = math.hypot(cxw - mwx, cyw - mwy)
                if d < best_d:
                    best_d = d
                    best = (gx, gy, mwx, mwy, gap)
            if best:
                clustered.append(best)
        # store clustered doors in grid coords with gap
        # format: (gx, gy, gap)
        self.ctx.raw_candidates = raw
        self.ctx.clustered_doors = [(gx, gy, gap) for (gx, gy, mwx, mwy, gap) in clustered]
        print(f"Detected {len(self.ctx.clustered_doors)} clustered doors.")
        self.state = State.DETECT_DOORS

    def handle_detect_doors(self):
        # simply move to selecting a goal
        self.state = State.SELECT_GOAL

    def handle_select_goal(self):
        if self.ctx.clustered_doors:
            # choose first detected doorway
            gx, gy, gap = self.ctx.clustered_doors[0]
            self.ctx.goal = (gx, gy)
            print(f"Selected door as goal at grid {gx,gy}, gap {gap:.2f}m")
        else:
            self.ctx.goal = self.ctx.ultimate_goal_grid
            print(f"No doors - using ultimate goal grid {self.ctx.goal}")
        self.state = State.PLAN_PATH

    def handle_plan_path(self):
        occ = self.ctx.occupancy_map
        start = (self.ctx.center_x, self.ctx.center_y)
        goal = self.ctx.goal
        if occ is None or start is None or goal is None:
            self.ctx.path = []
        else:
            self.ctx.path = bfs_path(occ, start, goal)
            if self.ctx.path:
                print(f"Planned path with {len(self.ctx.path)} nodes.")
            else:
                print("No path found.")
        self.state = State.DISPLAY

    def handle_display(self):
        self.ax.clear()
        occ = self.ctx.occupancy_map
        if occ is not None:
            self.ax.imshow(occ.T, cmap="gray_r", origin="lower")
        if self.ctx.center_x is not None:
            self.ax.plot(self.ctx.center_x, self.ctx.center_y, "og", label="Robot")
        # clustered doors
        for (gx, gy, gap) in (self.ctx.clustered_doors or []):
            self.ax.plot(gx, gy, "yo", markersize=6)
            self.ax.text(gx + 1, gy + 1, f"{gap:.2f}m", fontsize=6, color='y')
        # path
        if self.ctx.path:
            px, py = zip(*self.ctx.path)
            self.ax.plot(px, py, "b-", linewidth=1.5, label="Path")
        # ultimate goal
        ugx, ugy = self.ctx.ultimate_goal_grid
        self.ax.plot(ugx, ugy, "rx", markersize=8, label="Ultimate goal")
        self.ax.legend(loc='upper right')
        self.fig.canvas.draw_idle()
        plt.pause(UI_PAUSE)
        # increment scan index so next press will load next file
        self.ctx.scan_index += 1
        # go back to wait for user input
        self.state = State.WAIT_FOR_INPUT

# ---------------- Main ----------------
def main():
    ctx = Context()
    fsm = FSM(ctx)
    try:
        fsm.run()
    except KeyboardInterrupt:
        print("Interrupted by user")
        plt.close(fsm.fig)
        sys.exit(0)

if __name__ == "__main__":
    main()
