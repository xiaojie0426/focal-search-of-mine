import math
from typing import List, Tuple, Dict, Optional, Union
from obsracle_generate import ObstacleGenerator

class Node:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0
        self.focal_value = 0.0
        self.parent: Optional[Node] = None
        self.visited = False

    def __lt__(self, other: "Node"):
        if self.f != other.f:
            return self.f < other.f
        return self.g > other.g

    def __eq__(self, other: "Node"):
        return self.x == other.x and self.y == other.y


class GridMap:
    def __init__(self, width: int, height: int, obstacles: List[Tuple[int, int]]):
        """原有构造方法：通过障碍物列表初始化（保持兼容）"""
        self.width = width
        self.height = height
        self.grid = [[True for _ in range(width)] for _ in range(height)]  # True=可行，False=障碍物
        # 标记障碍物
        for (x, y) in obstacles:
            if 0 <= x < self.width and 0 <= y < self.height:
                self.grid[y][x] = False  # 注意：grid[y][x] 对应 (x,y) 栅格（y是行索引）

    @staticmethod
    def from_obstacle_generator(
        generator: ObstacleGenerator,
        gen_type: str = "count",  # 生成类型："count"（数量）、"density"（密度）、"clustered"（聚类）
        gen_value: Union[int, float] = 10,  # 对应类型的值：数量（int）、密度（float）
        seed: Optional[int] = None,
        **kwargs  # 聚类专用参数：cluster_num、cluster_radius
    ) -> "GridMap":
        """
        工厂方法：通过 ObstacleGenerator 直接创建带随机障碍物的地图（一行搞定）
        
        Args:
            generator: 障碍物生成器实例（已初始化地图尺寸、起点、终点）
            gen_type: 生成类型，可选 "count"（按数量）、"density"（按密度）、"clustered"（聚类）
            gen_value: 生成参数值：
                - gen_type="count" 时：int 类型，障碍物数量（如 10）
                - gen_type="density" 时：float 类型，障碍物密度（如 0.3 表示 30%）
                - gen_type="clustered" 时：int 类型，总障碍物数量（如 20）
            seed: 随机种子（固定后复现相同障碍物）
            **kwargs: 聚类专用参数（仅 gen_type="clustered" 时生效）：
                - cluster_num: 聚类数量（如 4，默认 3）
                - cluster_radius: 聚类半径（如 2，默认 2）
        
        Returns:
            GridMap 实例（含随机障碍物）
        """
        # 根据生成类型调用生成器的对应方法
        if gen_type == "count":
            # 按数量生成：gen_value 为障碍物数量（int）
            if not isinstance(gen_value, int):
                raise TypeError(f"gen_type='count' 时，gen_value 必须是整数，当前：{type(gen_value)}")
            obstacles = generator.generate(obstacle_count=gen_value, seed=seed)
        
        elif gen_type == "density":
            # 按密度生成：gen_value 为密度（float）
            if not isinstance(gen_value, float):
                raise TypeError(f"gen_type='density' 时，gen_value 必须是浮点数，当前：{type(gen_value)}")
            obstacles = generator.generate_by_density(density=gen_value, seed=seed)
        
        elif gen_type == "clustered":
            # 按聚类生成：gen_value 为总障碍物数量（int），需额外传 cluster_num 和 cluster_radius
            if not isinstance(gen_value, int):
                raise TypeError(f"gen_type='clustered' 时，gen_value 必须是整数，当前：{type(gen_value)}")
            cluster_num = kwargs.get("cluster_num", 3)
            cluster_radius = kwargs.get("cluster_radius", 2)
            obstacles = generator.generate_clustered(
                obstacle_count=gen_value,
                cluster_num=cluster_num,
                cluster_radius=cluster_radius,
                seed=seed
            )
        
        else:
            raise ValueError(f"不支持的 gen_type：{gen_type}，可选值：'count'/'density'/'clustered'")

        # 用生成器的地图尺寸 + 生成的障碍物创建 GridMap
        return GridMap(
            width=generator.map_width,
            height=generator.map_height,
            obstacles=obstacles
        )

    # -------------------------- 原有辅助方法（保持不变） --------------------------
    def is_valid(self, x: int, y: int) -> bool:
        """判断 (x,y) 栅格是否可行（在地图内 + 非障碍物）"""
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x]

    def get_neighbors(self, node: "Node", node_map: dict) -> List["Node"]:
        """获取节点的可行邻居（四方向移动，复用已存在节点）"""
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for dx, dy in directions:
            x, y = node.x + dx, node.y + dy
            if self.is_valid(x, y):
                key = (x, y)
                neighbor = node_map[key] if key in node_map else Node(x, y)
                neighbor.g = node.g + 1.0
                neighbors.append(neighbor)
        return neighbors