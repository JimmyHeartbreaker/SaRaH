import os
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# =========================
# --- LIDAR MAP SECTION ---
# =========================

EXTEND_AREA = 1.0

def file_read(f):
    with open(f) as data:
        measures = [line.split(",") for line in data]
    angles = [float(m[0]) for m in measures]
    distances = [float(m[1]) for m in measures]
    return np.array(angles), np.array(distances)

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
    occupancy_map = np.ones((x_w, y_w)) / 2
    center_x = int(round(-min_x / xy_resolution))
    center_y = int(round(-min_y / xy_resolution))
    for (x, y) in zip(ox, oy):
        ix = int(round((x - min_x) / xy_resolution))
        iy = int(round((y - min_y) / xy_resolution))
        laser_beams = bresenham((center_x, center_y), (ix, iy))
        for laser_beam in laser_beams:
            occupancy_map[laser_beam[0]][laser_beam[1]] = 0.0  # free area
        occupancy_map[ix][iy] = 1.0  # obstacle
    return occupancy_map, min_x, max_x, min_y, max_y, xy_resolution, center_x, center_y

# =========================
# --- A* PATH PLANNER ---
# =========================

class AStarPlanner:
    def __init__(self, obstacle_map, resolution):
        self.resolution = resolution
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
                print("No path found")
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

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

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
            [1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
            [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]
        ]

# =========================
# --- MAIN ---
# =========================

def main():
    # CSV file path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file = os.path.join(script_dir, "lidar01.csv")

    xy_resolution = 0.02
    ang, dist = file_read(csv_file)
    ox = np.sin(ang) * dist
    oy = np.cos(ang) * dist

    # Generate map and get LIDAR origin
    occupancy_map, min_x, max_x, min_y, max_y, xy_resolution, center_x, center_y = \
        generate_ray_casting_grid_map(ox, oy, xy_resolution)

    # Start point = LIDAR origin
    sx, sy = center_x, center_y
    print(f"Start (LIDAR origin) at: {sx}, {sy}")

    # Click to select goal point
    coords = []
    def onclick(event):
        ix, iy = int(event.xdata), int(event.ydata)
        print(f"Goal clicked: {ix}, {iy}")
        coords.append((ix, iy))
        plt.close()

    plt.imshow(occupancy_map.T, cmap="gray_r", origin="lower")
    plt.plot(sx, sy, "og", label="Start (LIDAR)")
    plt.title("Click GOAL point")
    plt.show(block=False)
    cid = plt.gcf().canvas.mpl_connect('button_press_event', onclick)
    plt.show()

    if len(coords) < 1:
        print("No goal selected, exiting")
        return

    gx, gy = coords[0]

    # Run A*
    a_star = AStarPlanner(occupancy_map, xy_resolution)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    # Plot final path
    plt.imshow(occupancy_map.T, cmap="gray_r", origin="lower")
    plt.plot(sx, sy, "og", label="Start (LIDAR)")
    plt.plot(gx, gy, "xb", label="Goal")
    if rx:
        plt.plot(rx, ry, "-r", label="A* Path")
    plt.legend()
    plt.title("A* Path on LIDAR Occupancy Map")
    plt.show()

if __name__ == '__main__':
    main()
