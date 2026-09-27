"""寻路 - 带5个bug"""
import heapq
from dataclasses import dataclass, field

@dataclass
class Grid:
    width: int
    height: int
    obstacles: set = field(default_factory=set)

    def is_walkable(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height and (x, y) not in self.obstacles

    def neighbors(self, x, y):
        # 八方向
        dirs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        return [(x+dx, y+dy) for dx, dy in dirs if self.is_walkable(x+dx, y+dy)]

class Pathfinder:
    def __init__(self, grid: Grid):
        self.grid = grid
        self.open_list: list = []  # bug2: 普通列表
        self.affected_regions: set = set()  # bug3: 不标记影响区域

    def heuristic(self, a, b):
        # bug1: 八方向用曼哈顿距离，不可容许
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def find_path(self, start, goal):
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path
            for neighbor in self.grid.neighbors(*current):
                tentative = g_score[current] + 1
                if neighbor not in g_score or tentative < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f = tentative + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
        return []

    def on_obstacle_moved(self, old_pos, new_pos):
        # bug3: 全图重算，不标记影响区域
        self.affected_regions = set()

    def smooth_path(self, path):
        # bug4: 简单直线连接，不检查碰撞
        if len(path) < 3:
            return path
        return [path[0], path[-1]]

    def group_flocking(self, units, target):
        # bug5: 简单排斥力，窄通道互推
        results = []
        for u in units:
            repel = [0, 0]
            for other in units:
                if other is not u:
                    dx = u[0] - other[0]
                    dy = u[1] - other[1]
                    dist = (dx**2 + dy**2)**0.5
                    if dist < 2:
                        repel[0] += dx / dist
                        repel[1] += dy / dist
            results.append((u[0] + repel[0] * 0.5, u[1] + repel[1] * 0.5))
        return results
