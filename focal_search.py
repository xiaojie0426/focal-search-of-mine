from typing import List, Tuple, Dict, Optional
import heapq
from env import Node, GridMap
import math
import random

import heapq
import random
from typing import List, Tuple, Dict, Optional, Callable




class FocalSearch:
    def __init__(self, grid_map, w: float = 1.2):
        self.grid_map = grid_map
        self.w = w
        self.open = []
        self.focal = []
        self.node_map: Dict[Tuple[int, int], Node] = {}
        self.f_min = float('inf')

    def _calculate_heuristic(self, node: Node, goal: Node) -> float:
        # 曼哈顿距离
        return abs(node.x - goal.x) + abs(node.y - goal.y)

    def _dir_consistency(self, node: Node, goal: Node) -> float:
        if node.parent is None:
            return 0.0
        dir_parent = (node.x - node.parent.x, node.y - node.parent.y)
        dir_goal = (
            0 if node.x == goal.x else (1 if node.x < goal.x else -1),
            0 if node.y == goal.y else (1 if node.y < goal.y else -1)
        )
        return 1.0 if dir_parent == dir_goal else 0.0

    def _calculate_focal_value(self, node: Node, goal: Node) -> float:
        """默认的二次排序值计算"""
        dir_consistency = self._dir_consistency(node, goal)
        max_g = self.grid_map.width + self.grid_map.height
        normalized_g = node.g / max_g if max_g != 0 else 0.0
        # 因为排序时希望较小的 focal_value 优先扩展（可以改成直接取负数排序）
        return 1.0 - (0.6 * dir_consistency + 0.4 * (1 - normalized_g))

    def _update_focal(self):
        """更新 Focal 集合"""
        self.focal = []
        if not self.open:
            return
        unvisited_nodes = [n for n in self.open if not n.visited]
        if not unvisited_nodes:
            return
        self.f_min = min(n.f for n in unvisited_nodes)
        self.focal = [n for n in unvisited_nodes if n.f <= (self.w +1) * self.f_min]
        self.focal.sort(key=lambda x: x.focal_value)

    def _backtrack_path(self, goal_node: Node) -> List[Tuple[int, int]]:
        path = []
        current = goal_node
        while current:
            path.append((current.x, current.y))
            current = current.parent
        return path[::-1]

    def search_once(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """单次搜索，返回一条路径"""
        start_node = Node(start[0], start[1])
        goal_node = Node(goal[0], goal[1])

        start_node.h = self._calculate_heuristic(start_node, goal_node)
        start_node.f = start_node.g + start_node.h
        start_node.focal_value = self._calculate_focal_value(start_node, goal_node)

        self.open = []
        heapq.heappush(self.open, start_node)
        self.node_map = {(start[0], start[1]): start_node}
        self.f_min = start_node.f

        while self.open:
            self._update_focal()
            if not self.focal:
                continue

            current = self.focal[0]
            current.visited = True

            if current == goal_node:
                return self._backtrack_path(current)

            neighbors = self.grid_map.get_neighbors(current, self.node_map)
            for neighbor in neighbors:
                if neighbor.visited:
                    continue
                neighbor.h = self._calculate_heuristic(neighbor, goal_node)
                new_f = neighbor.g + neighbor.h
                key = (neighbor.x, neighbor.y)

                if key not in self.node_map:
                    neighbor.f = new_f
                    neighbor.focal_value = self._calculate_focal_value(neighbor, goal_node)
                    neighbor.parent = current
                    self.node_map[key] = neighbor
                    heapq.heappush(self.open, neighbor)
                    if neighbor.f < self.f_min:
                        self.f_min = neighbor.f
                else:# 可能出现一条路径重复经过两次同一个节点的情况
                    existing = self.node_map[key]
                    if neighbor.g < existing.g:
                        existing.g = neighbor.g
                        existing.f = existing.g + existing.h
                        existing.focal_value = self._calculate_focal_value(existing, goal_node)
                        existing.parent = current
                        heapq.heappush(self.open, existing)
                        if existing.f < self.f_min:
                            self.f_min = existing.f

        return None

    def generate_candidate_paths(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        candidate_num: int = 3,
        max_tries: int = 20,
        w_min: float = 1.0,
        w_max: float = 3.0,
        noise_strength: float = 0.01
    ) -> List[List[Tuple[int, int]]]:
        """
        稳定生成指定数量的候选路径
        :param candidate_num: 需要生成的路径数量
        :param max_tries: 最大尝试次数
        :param w_min: 次优系数最小值
        :param w_max: 次优系数最大值
        :param noise_strength: 启发式函数噪声强度
        """
        candidate_paths = []
        used_paths = set()

        original_heuristic = self._calculate_heuristic
        original_focal_calc = self._calculate_focal_value

        for _ in range(max_tries):
            if len(candidate_paths) >= candidate_num:
                break

            # 随机 w
            self.w = random.uniform(w_min, w_max)

            # 随机调整二次排序权重
            dir_weight = random.uniform(0.3, 0.9)
            g_weight = 1.0 - dir_weight

            def new_focal_calc(node: Node, goal: Node) -> float:
                dir_consistency = self._dir_consistency(node, goal)
                max_g = self.grid_map.width + self.grid_map.height
                normalized_g = node.g / max_g if max_g != 0 else 0.0
                return 1.0 - (dir_weight * dir_consistency + g_weight * (1 - normalized_g))

            self._calculate_focal_value = new_focal_calc

            # 启发式加噪声
            def noisy_heuristic(node: Node, goal: Node) -> float:
                return original_heuristic(node, goal) + random.uniform(-noise_strength, noise_strength)

            self._calculate_heuristic = noisy_heuristic

            # 搜索一次
            path = self.search_once(start, goal)

            # 恢复原函数
            self._calculate_heuristic = original_heuristic
            self._calculate_focal_value = original_focal_calc

            if path:
                path_tuple = tuple(path)
                if path_tuple not in used_paths:
                    used_paths.add(path_tuple)
                    candidate_paths.append(path)

        return candidate_paths