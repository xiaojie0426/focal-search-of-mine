好的，我来帮你**逐句解析 FocalSearch 类**的代码，这样你就能完全理解它的运行原理。  
我会结合 **Focal Search 算法原理** 来解释每一部分的作用。

---

## **Focal Search 核心思想回顾**
Focal Search 是 A* 算法的一个变种，主要改进点：
1. 除了 **Open list**（按 \( f(n) = g(n) + h(n) \) 排序），还维护一个 **Focal list**；
2. Focal list 是 Open list 中 \( f(n) \le (1 + w) \cdot f_{\min} \) 的节点集合（w 是次优系数）；
3. 在 Focal list 中，不是按 \( f(n) \) 扩展，而是按**二次评估函数**（focal value）排序扩展；
4. 这样可以在保证接近最优的前提下，灵活调整搜索方向，从而生成不同路径（多样性）。

---

## **代码解析**

```python
class FocalSearch:
    def __init__(self, grid_map, w: float = 1.2):
        self.grid_map = grid_map  # 栅格地图对象
        self.w = w  # 次优系数（控制 Focal 集合范围）
        self.open = []  # Open list（优先队列，按 f 排序）
        self.focal = []  # Focal list（按 focal_value 排序）
        self.node_map: Dict[Tuple[int, int], Node] = {}  # 记录已创建的节点
        self.f_min = float('inf')  # 当前 Open list 中最小的 f 值
```

**解析：**
- `grid_map`：保存地图信息（障碍物、尺寸等）；
- `w`：次优系数，控制 Focal 集合的范围（w 越大，Focal 集合越大，探索范围越广）；
- `open`：存放待扩展节点，按 \( f \) 值排序；
- `focal`：存放从 Open list 筛选出的“次优可接受”节点，按 `focal_value` 排序；
- `node_map`：用坐标 `(x, y)` 映射 Node 对象，避免重复创建；
- `f_min`：记录当前 Open list 中最小的 f 值，用于计算 Focal 集合范围。

---

### **启发式函数**
```python
def _calculate_heuristic(self, node: Node, goal: Node) -> float:
    # 曼哈顿距离
    return abs(node.x - goal.x) + abs(node.y - goal.y)
```
**解析：**
- 这里用**曼哈顿距离**作为启发式函数 \( h(n) \)，适合四方向移动的栅格地图；
- 公式：
\[
h(n) = |x_n - x_{goal}| + |y_n - y_{goal}|
\]
- 作用：估计从当前节点到终点的最短代价，影响 A* 的搜索效率和 Focal 集合的构成。

---

### **方向一致性**
```python
def _dir_consistency(self, node: Node, goal: Node) -> float:
    if node.parent is None:
        return 0.0
    dir_parent = (node.x - node.parent.x, node.y - node.parent.y)  # 从父节点到当前节点的方向
    dir_goal = (
        0 if node.x == goal.x else (1 if node.x < goal.x else -1),
        0 if node.y == goal.y else (1 if node.y < goal.y else -1)
    )  # 当前节点指向终点的期望方向
    return 1.0 if dir_parent == dir_goal else 0.0
```
**解析：**
- 判断当前节点的**前进方向**是否与“指向终点的理想方向”一致；
- 如果一致，返回 1.0，否则返回 0.0；
- 这是**二次排序**的一个特征值，用来引导搜索朝终点方向前进，也可用于生成多样化路径。

---

### **Focal 二次排序值计算**
```python
def _calculate_focal_value(self, node: Node, goal: Node) -> float:
    """默认的二次排序值计算"""
    dir_consistency = self._dir_consistency(node, goal)
    max_g = self.grid_map.width + self.grid_map.height
    normalized_g = node.g / max_g if max_g != 0 else 0.0
    return 1.0 - (0.6 * dir_consistency + 0.4 * (1 - normalized_g))
```
**解析：**
- Focal 集合内的节点不按 f 值排序，而是按 `focal_value` 排序；
- 这里的公式综合了：
  1. **方向一致性**（权重 0.6）：鼓励朝终点方向前进；
  2. **已走代价 g**（权重 0.4）：已走的步数越多，该项越大，使搜索倾向于尽快到达终点；
- `1.0 - (...)` 是因为排序时希望较小的 focal_value 优先扩展（可以改成直接取负数排序）。

---

### **更新 Focal 集合**
```python
def _update_focal(self):
    """更新 Focal 集合"""
    self.focal = []
    if not self.open:
        return
    unvisited_nodes = [n for n in self.open if not n.visited]
    if not unvisited_nodes:
        return
    self.f_min = min(n.f for n in unvisited_nodes)  # 当前 Open list 最小 f 值
    # 筛选 f(n) <= (1 + w) * f_min 的节点进入 Focal 集合
    self.focal = [n for n in unvisited_nodes if n.f <= (1 + self.w) * self.f_min]
    # 按 focal_value 排序
    self.focal.sort(key=lambda x: x.focal_value)
```
**解析：**
- 先找出 Open list 中未访问的节点；
- 计算当前最小 f 值 `f_min`；
- 按公式 \( f(n) \le (1 + w) \cdot f_{\min} \) 筛选节点进入 Focal 集合；
- 对 Focal 集合按 `focal_value` 排序，决定扩展顺序。

---

### **回溯路径**
```python
def _backtrack_path(self, goal_node: Node) -> List[Tuple[int, int]]:
    path = []
    current = goal_node
    while current:
        path.append((current.x, current.y))
        current = current.parent
    return path[::-1]
```
**解析：**
- 从目标节点 `goal_node` 沿着 `parent` 指针回溯到起点；
- 用 `[::-1]` 反转列表，得到从起点到终点的路径。

---

### **单次搜索**
```python
def search_once(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
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

        current = self.focal[0]  # 从 Focal 集合取最优节点
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
            else:
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
```
**解析：**
1. 初始化起点和终点节点；
2. 计算起点的 h、f、focal_value，加入 Open list；
3. 循环：
   - 更新 Focal 集合；
   - 从 Focal 集合取 `focal_value` 最小的节点扩展；
   - 如果是终点，回溯路径并返回；
   - 遍历邻居节点，更新 g、h、f 和 focal_value；
   - 如果是新节点，加入 Open list；
   - 如果找到更优路径，更新已有节点信息；
4. 如果 Open list 为空还没找到终点，返回 None（无解）。

---

### **生成多条候选路径**
```python
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
    candidate_paths = []
    used_paths = set()

    original_heuristic = self._calculate_heuristic
    original_focal_calc = self._calculate_focal_value

    for _ in range(max_tries):
        if len(candidate_paths) >= candidate_num:
            break

        self.w = random.uniform(w_min, w_max)  # 随机 w

        dir_weight = random.uniform(0.3, 0.9)
        g_weight = 1.0 - dir_weight

        def new_focal_calc(node: Node, goal: Node) -> float:
            dir_consistency = self._dir_consistency(node, goal)
            max_g = self.grid_map.width + self.grid_map.height
            normalized_g = node.g / max_g if max_g != 0 else 0.0
            return 1.0 - (dir_weight * dir_consistency + g_weight * (1 - normalized_g))

        self._calculate_focal_value = new_focal_calc

        def noisy_heuristic(node: Node, goal: Node) -> float:
            return original_heuristic(node, goal) + random.uniform(-noise_strength, noise_strength)

        self._calculate_heuristic = noisy_heuristic

        path = self.search_once(start, goal)

        self._calculate_heuristic = original_heuristic
        self._calculate_focal_value = original_focal_calc

        if path:
            path_tuple = tuple(path)
            if path_tuple not in used_paths:
                used_paths.add(path_tuple)
                candidate_paths.append(path)

    return candidate_paths
```
**解析：**
- 通过多次运行 `search_once` 生成多条路径；
- 每次运行前随机调整：
  1. `w`（Focal 集合范围）；
  2. 二次排序权重（`dir_weight` 和 `g_weight`）；
  3. 启发式函数加随机噪声；
- 这样每次搜索的探索方向略有不同，可能得到不同路径；
- 用 `used_paths` 集合去重，确保路径不重复。

---

✅ **总结**：
- `w` 控制 Focal 集合范围，影响搜索多样性；
- `_update_focal()` 是 Focal Search 的核心，负责筛选次优节点集合；
- `_calculate_focal_value()` 决定 Focal 集合内的扩展顺序；
- `generate_candidate_paths()` 通过改变搜索参数，生成多条不同路径。

---

如果你愿意，我可以帮你**画一张 Focal Search 流程图**，直观展示 Open list、Focal list 如何交互，这样你调参时会更有感觉。  
要帮你画吗？这样你能更直观理解整个类的运行机制。# focal-search-of-mine
