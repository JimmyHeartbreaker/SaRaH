# path_planner.py
import math

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
