# doorway_detection.py
import math
import numpy as np
from config import DOOR_THRESHOLD, CLUSTER_EPS, CLUSTER_MIN_SAMPLES, RING_RADIUS_M, SAMPLE_COUNT

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

    segments, current = [], []
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
