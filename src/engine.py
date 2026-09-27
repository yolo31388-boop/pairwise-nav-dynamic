"""寻路 - 修复版"""
import heapq
import itertools
import math
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
    REPLAN_RADIUS = 2  # 动态障碍影响半径

    def __init__(self, grid: Grid):
        self.grid = grid
        self.open_list: list = []  # 堆，支持递减键（懒惰删除）
        self.affected_regions: set = set()

    def heuristic(self, a, b):
        # 八方向地图用切比雪夫距离，可容许且一致
        return max(abs(a[0]-b[0]), abs(a[1]-b[1]))

    def find_path(self, start, goal):
        # 开放列表：堆 + 递减键（发现更优 g 时压入新条目，旧条目弹出时判过期丢弃）
        self.open_list = []
        counter = itertools.count()
        came_from = {}
        g_score = {start: 0}
        best_f = {start: self.heuristic(start, goal)}
        heapq.heappush(self.open_list, (best_f[start], next(counter), start))
        while self.open_list:
            f, _, current = heapq.heappop(self.open_list)
            if f > best_f.get(current, math.inf):
                continue  # 过期条目，已被更优键替代
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            for neighbor in self.grid.neighbors(*current):
                tentative = g_score[current] + 1
                if tentative < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    nf = tentative + self.heuristic(neighbor, goal)
                    best_f[neighbor] = nf
                    heapq.heappush(self.open_list, (nf, next(counter), neighbor))
        return []

    def on_obstacle_moved(self, old_pos, new_pos):
        # 只标记旧位置和新位置附近的影响区域，区域外的路径不重算
        self.affected_regions = set()
        for cx, cy in (old_pos, new_pos):
            for dx in range(-self.REPLAN_RADIUS, self.REPLAN_RADIUS + 1):
                for dy in range(-self.REPLAN_RADIUS, self.REPLAN_RADIUS + 1):
                    x, y = cx + dx, cy + dy
                    if 0 <= x < self.grid.width and 0 <= y < self.grid.height:
                        self.affected_regions.add((x, y))

    def path_needs_replan(self, path):
        # 只有穿过影响区域的路径才需要重算
        return any(p in self.affected_regions for p in path)

    def _line_of_sight(self, a, b):
        # 沿线段采样，检查是否穿过障碍
        steps = max(abs(b[0]-a[0]), abs(b[1]-a[1]), 1)
        for i in range(steps + 1):
            t = i / steps
            x = round(a[0] + (b[0]-a[0]) * t)
            y = round(a[1] + (b[1]-a[1]) * t)
            if not self.grid.is_walkable(x, y):
                return False
        return True

    def smooth_path(self, path):
        # 直线拉通，但逐段检查碰撞；穿障碍的段保留原始路径点
        if len(path) < 3:
            return list(path)
        smoothed = [path[0]]
        anchor = 0
        i = 2
        while i < len(path):
            if self._line_of_sight(path[anchor], path[i]):
                i += 1
            else:
                smoothed.append(path[i-1])
                anchor = i - 1
                i = anchor + 2
        smoothed.append(path[-1])
        return smoothed

    def group_flocking(self, units, target):
        # 窄通道排队：离目标近的单位优先级高先走，后方单位减速等待让行
        def dist(a, b):
            return math.hypot(a[0]-b[0], a[1]-b[1])

        MIN_GAP = 0.8
        order = sorted(range(len(units)), key=lambda i: dist(units[i], target))
        results = [None] * len(units)
        for rank, idx in enumerate(order):
            ux, uy = units[idx]
            dx, dy = target[0]-ux, target[1]-uy
            d = math.hypot(dx, dy)
            vx, vy = (dx/d, dy/d) if d > 1e-9 else (0.0, 0.0)
            speed = 1.0
            for j in range(rank):
                ahead = results[order[j]]
                gap = dist((ux, uy), ahead)
                if gap < MIN_GAP * 0.5:
                    speed = 0.0  # 紧贴前车，停车等待
                    break
                elif gap < MIN_GAP:
                    speed = min(speed, 0.3)  # 减速跟随
            results[idx] = (ux + vx*speed, uy + vy*speed)
        return results
