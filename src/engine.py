"""寻路 - 修复版"""
import heapq
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
        # 八方向，禁止对角穿墙角
        dirs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        result = []
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if not self.is_walkable(nx, ny):
                continue
            if dx != 0 and dy != 0 and not (
                self.is_walkable(x + dx, y) and self.is_walkable(x, y + dy)
            ):
                continue
            result.append((nx, ny))
        return result

class Pathfinder:
    def __init__(self, grid: Grid):
        self.grid = grid
        self.open_list: list = []  # 二叉堆，支持递减键（懒删除）
        self.affected_regions: set = set()

    def heuristic(self, a, b):
        # 八方向 + 对角代价sqrt(2)，欧几里得距离可容许且一致
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def find_path(self, start, goal):
        if not self.grid.is_walkable(*start) or not self.grid.is_walkable(*goal):
            return []
        open_heap = [(self.heuristic(start, goal), 0.0, start)]
        self.open_list = open_heap
        came_from = {}
        g_score = {start: 0.0}
        closed = set()
        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:  # 懒删除：跳过已被更优键替换的旧堆项
                continue
            closed.add(current)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            for neighbor in self.grid.neighbors(*current):
                if neighbor in closed:
                    continue
                step = math.hypot(neighbor[0] - current[0], neighbor[1] - current[1])
                tentative = g_score[current] + step
                if tentative < g_score.get(neighbor, math.inf):
                    # 递减键：直接压入更小的键，旧项出堆时被丢弃
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f = tentative + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f, tentative, neighbor))
        return []

    def on_obstacle_moved(self, old_pos, new_pos, radius=2):
        # 只标记旧位置和新位置附近的影响区域，区域外的路径不重算
        self.affected_regions = set()
        for cx, cy in (old_pos, new_pos):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    x, y = cx + dx, cy + dy
                    if 0 <= x < self.grid.width and 0 <= y < self.grid.height:
                        self.affected_regions.add((x, y))
        if old_pos in self.grid.obstacles:
            self.grid.obstacles.discard(old_pos)
            self.grid.obstacles.add(new_pos)
        return self.affected_regions

    def path_needs_replan(self, path):
        # 只有穿过影响区域的路径才需要重算
        return any(p in self.affected_regions for p in path)

    def smooth_path(self, path):
        # 视线平滑：逐段检查碰撞，穿障碍的段保留原始路径点
        if len(path) < 3:
            return list(path)
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._line_of_sight(path[i], path[j]):
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed

    def _line_of_sight(self, a, b):
        x0, y0 = a
        x1, y1 = b
        steps = int(max(abs(x1 - x0), abs(y1 - y0)) * 4) + 1
        for k in range(steps + 1):
            t = k / steps
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            if not self.grid.is_walkable(round(x), round(y)):
                return False
        return True

    def group_flocking(self, units, target, speed=0.5, min_gap=0.8):
        # 窄通道排队：离目标越近让行优先级越高，后方单位减速等待
        order = sorted(
            range(len(units)),
            key=lambda i: (units[i][0] - target[0]) ** 2 + (units[i][1] - target[1]) ** 2,
        )
        results = [None] * len(units)
        for idx in order:
            ux, uy = units[idx]
            dx, dy = target[0] - ux, target[1] - uy
            dist = math.hypot(dx, dy)
            if dist == 0:
                results[idx] = (ux, uy)
                continue
            dir_x, dir_y = dx / dist, dy / dist
            step = min(speed, dist)
            # 前方有更高优先级的单位时减速排队，保持跟随间距
            for ahead in order:
                if ahead == idx:
                    break
                ox, oy = results[ahead] if results[ahead] is not None else units[ahead]
                rel_x, rel_y = ox - ux, oy - uy
                if rel_x * dir_x + rel_y * dir_y <= 0:
                    continue  # 不在正前方，不阻塞
                gap = math.hypot(rel_x, rel_y)
                step = min(step, max(0.0, gap - min_gap))
            results[idx] = (ux + dir_x * step, uy + dir_y * step)
        return results
