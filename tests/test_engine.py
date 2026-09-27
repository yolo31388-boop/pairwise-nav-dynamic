"""寻路测试 - 7个失败"""
import pytest, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from src.engine import Grid, Pathfinder

class TestHeuristic:
    def test_octile_heuristic_admissible(self):
        grid = Grid(10, 10)
        pf = Pathfinder(grid)
        h = pf.heuristic((0,0), (3,4))
        assert h <= 5, f"启发不可容许: {h}"

class TestPathOptimal:
    def test_path_is_optimal(self):
        grid = Grid(10, 10)
        pf = Pathfinder(grid)
        path = pf.find_path((0,0), (3,4))
        assert len(path) == 5, f"非最优路径: {len(path)}"

class TestDynamicObstacle:
    def test_affected_region_marking(self):
        grid = Grid(10, 10)
        pf = Pathfinder(grid)
        pf.on_obstacle_moved((2,2), (3,3))
        assert len(pf.affected_regions) > 0, "影响区域未标记"

class TestPathSmoothing:
    def test_smoothed_path_no_collision(self):
        grid = Grid(10, 10, obstacles={(5, 5)})
        pf = Pathfinder(grid)
        path = pf.find_path((0,0), (9,9))
        smoothed = pf.smooth_path(path)
        for p in smoothed:
            assert grid.is_walkable(*p), f"平滑后穿墙: {p}"

class TestFlocking:
    def test_narrow_corridor_no_deadlock(self):
        pf = Pathfinder(Grid(10, 10))
        units = [(5, 5), (5, 5.1), (5, 5.2)]
        result = pf.group_flocking(units, (5, 10))
        assert len(result) == 3
