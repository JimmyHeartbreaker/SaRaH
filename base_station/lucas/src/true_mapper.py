import math
import time
from typing import Tuple
import numpy as np
from mapper.PlannerWHistory import cluster_points, detect_doorways_in_grid, generate_ray_casting_grid_map

XY_RESOLUTION = 0.02 

class TrueMapperResults:
    occupancy_map:list[Tuple[int, int]] =None
    center_x:float = None
    center_y:float= None
    min_x:float = None
    min_y:float=None
    clustered_doors:list=None
    door_history:list=None
    pass

def true_mapper(points, scan_index):
    results = TrueMapperResults() 
 #CSV has been read
    ox = np.array([p.x for p in points])
    oy = np.array([p.y for p in points])

    occ_map, cx, cy, min_x, min_y, x_w, y_w = generate_ray_casting_grid_map(
        ox, oy, XY_RESOLUTION)

    raw_candidates = detect_doorways_in_grid(
        occ_map, cx, cy, min_x, min_y, XY_RESOLUTION)

    pts = [(mwx, mwy) for (mwx, mwy, gap, gx, gy) in raw_candidates]
    clusters = cluster_points(pts)

    clustered = []
    results.center_x  = cx
    results.center_y = cy
    results.min_x = min_x
    results.min_y = min_y
    door_history = []    
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

        results.occupancy_map = occ_map
        
        results.clustered_doors = clustered

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
    results.door_history = door_history
    print(f"Scan loaded. {len(clustered)} doors detected.")
    return results