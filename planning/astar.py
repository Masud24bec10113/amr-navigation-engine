import heapq
import numpy as np


class AStarPlanner:
    def __init__(self, grid_size=30):
        self.size = grid_size

    def plan(self, start, goal, obstacles, safety_margin=1.2):
        start_node = (int(round(start[0])), int(round(start[1])))
        goal_node = (int(round(goal[0])), int(round(goal[1])))

        def is_collision(x, y):
            for ox, oy, r in obstacles:
                if np.hypot(x - ox, y - oy) <= (r + safety_margin):
                    return True
            return False

        if is_collision(*start_node) or is_collision(*goal_node):
            raise ValueError("Start or Goal coordinate intersects an obstacle safety zone.")

        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0.0}
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_node:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return np.array(path, dtype=np.float64)

            for dx, dy in neighbors:
                nbr = (current[0] + dx, current[1] + dy)
                if 0 <= nbr[0] < self.size and 0 <= nbr[1] < self.size:
                    if is_collision(nbr[0], nbr[1]):
                        continue
                    tentative_g = g_score[current] + np.hypot(dx, dy)
                    if tentative_g < g_score.get(nbr, float('inf')):
                        came_from[nbr] = current
                        g_score[nbr] = tentative_g
                        f_score = tentative_g + np.linalg.norm(np.array(nbr) - np.array(goal_node))
                        heapq.heappush(open_set, (f_score, nbr))
        raise RuntimeError("No path found.")
