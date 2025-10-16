import random
from typing import List, Tuple, Optional, Union

class ObstacleGenerator:
    def __init__(
        self,
        map_width: int,
        map_height: int,
        start: Tuple[int, int],
        goal: Tuple[int, int]
    ):
        # 校验地图尺寸
        if map_width <= 0 or map_height <= 0:
            raise ValueError(f"地图尺寸必须为正整数，当前：宽={map_width}, 高={map_height}")
        self.map_width = map_width
        self.map_height = map_height

        # 校验起点/终点合法性
        self._validate_point(start, "起点")
        self._validate_point(goal, "终点")
        self.start = start
        self.goal = goal

        # 预计算关键参数
        self.total_grids = self.map_width * self.map_height
        self.max_obstacle_count = self.total_grids - 2  # 避开创点/终点
        self.valid_grids = self._get_all_valid_grids()  # 预生成可行栅格列表

    def _validate_point(self, point: Tuple[int, int], point_name: str):
        x, y = point
        if not (0 <= x < self.map_width and 0 <= y < self.map_height):
            raise ValueError(
                f"{point_name} ({x},{y}) 超出地图范围！"
                f"x∈[0,{self.map_width-1}], y∈[0,{self.map_height-1}]"
            )

    def _get_all_valid_grids(self) -> List[Tuple[int, int]]:
        """生成所有“非起点/非终点”的可行栅格"""
        return [
            (x, y) for y in range(self.map_height)
            for x in range(self.map_width)
            if (x, y) != self.start and (x, y) != self.goal
        ]

    def _validate_obstacle_count(self, count: int):
        if count < 0:
            raise ValueError(f"障碍物数量不能为负，当前：{count}")
        if count > self.max_obstacle_count:
            raise ValueError(
                f"障碍物数量超出上限！当前：{count}，最大允许：{self.max_obstacle_count}"
            )

    # 核心生成方法：按数量
    def generate(self, obstacle_count: int, seed: Optional[int] = None) -> List[Tuple[int, int]]:
        self._validate_obstacle_count(obstacle_count)
        if obstacle_count == 0:
            return []
        random.seed(seed) if seed is not None else random.seed()
        return random.sample(self.valid_grids, k=obstacle_count)

    # 扩展生成方法：按密度
    def generate_by_density(self, density: float, seed: Optional[int] = None) -> List[Tuple[int, int]]:
        if not (0.0 <= density <= 1.0):
            raise ValueError(f"密度必须在 [0.0,1.0] 内，当前：{density}")
        if density == 0.0:
            return []
        target_count = min(int(self.max_obstacle_count * density), self.max_obstacle_count)
        return self.generate(obstacle_count=target_count, seed=seed)

    # 扩展生成方法：聚类障碍物（模拟墙壁/货架）
    def generate_clustered(
        self,
        obstacle_count: int,
        cluster_num: int = 3,
        cluster_radius: int = 2,
        seed: Optional[int] = None
    ) -> List[Tuple[int, int]]:
        self._validate_obstacle_count(obstacle_count)
        if cluster_num <= 0 or cluster_radius <= 0:
            raise ValueError("聚类数量/半径必须为正整数")
        
        random.seed(seed) if seed is not None else random.seed()
        cluster_centers = random.sample(self.valid_grids, k=cluster_num)
        obstacles = set()

        # 围绕中心生成聚类障碍物
        for (cx, cy) in cluster_centers:
            if len(obstacles) >= obstacle_count:
                break
            for dx in range(-cluster_radius, cluster_radius + 1):
                for dy in range(-cluster_radius, cluster_radius + 1):
                    x, y = cx + dx, cy + dy
                    grid = (x, y)
                    if grid in self.valid_grids and grid not in obstacles:
                        obstacles.add(grid)
                        if len(obstacles) >= obstacle_count:
                            break
                if len(obstacles) >= obstacle_count:
                    break

        # 补充不足数量的随机障碍物
        if len(obstacles) < obstacle_count:
            remaining = obstacle_count - len(obstacles)
            remaining_grids = [g for g in self.valid_grids if g not in obstacles]
            obstacles.update(random.sample(remaining_grids, k=remaining))
        
        return list(obstacles)

