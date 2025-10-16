import heapq
import matplotlib.pyplot as plt
import math
from typing import List, Tuple, Dict, Optional
from env import Node, GridMap
from focal_search import FocalSearch
from plot import print_grid_map, visualize_grid_and_paths
from obsracle_generate import ObstacleGenerator



if __name__ == "__main__":
    # -------------------------- 1. 初始化障碍物生成器（基础配置） --------------------------
    # 地图尺寸、起点、终点（只需配置一次）
    map_width = 20
    map_height = 20
    start = (15, 18)
    goal = (0, 4)

    # 创建生成器实例
    try:
        obs_generator = ObstacleGenerator(
            map_width=map_width,
            map_height=map_height,
            start=start,
            goal=goal
        )
    except ValueError as e:
        print(f"生成器初始化失败：{e}")
        exit()

    # -------------------------- 2. 用工厂方法生成地图（三种场景任选） --------------------------
    # 场景1：按数量生成（15个障碍物，固定种子42，复现相同障碍物）
    '''grid_map = GridMap.from_obstacle_generator(
        generator=obs_generator,
        gen_type="count",
        gen_value=15,  # 15个障碍物
        seed=42
    )'''

    # # 场景2：按密度生成（30%密度，随机种子，每次不同）
    grid_map = GridMap.from_obstacle_generator(
        generator=obs_generator,
        gen_type="density",
        gen_value=0.1,  # 30%栅格是障碍物
        seed=1
    )

    # # 场景3：聚类生成（20个障碍物，4个聚类块，半径2，固定种子100）
    '''grid_map = GridMap.from_obstacle_generator(
        generator=obs_generator,
        gen_type="clustered",
        gen_value=20,  # 总障碍物数量
        seed=100,
        cluster_num=4,  # 4个障碍物块
        cluster_radius=2  # 每个块3×3栅格
    )'''

    # -------------------------- 3. 验证地图并生成候选路径 --------------------------
    # 打印地图确认障碍物
    # print_grid_map(grid_map, start, goal)

    # 生成Focal Search候选路径
    focal = FocalSearch(grid_map, w=0.2)
    candidate_paths  = focal.generate_candidate_paths(
    start=start,
    goal=goal,
    candidate_num=20,
    max_tries=100,
    w_min=0,
    w_max=3.0
)

    # -------------------------- 4. 可视化结果 --------------------------
    if candidate_paths:
        print(f"\n成功生成 {len(candidate_paths)} 条候选路径：")
        for i, path in enumerate(candidate_paths):
            # 简化打印（太长时省略中间部分）
            path_str = f"[{path[0]} → ... → {path[-1]}]"
            # path_str = " → ".join([f"({x},{y})" for x, y in path])
            print(f"路径{i+1}（长度：{len(path)-1}）：{path_str}")
        # 可视化
        visualize_grid_and_paths(
            grid_map=grid_map,
            start=start,
            goal=goal,
            candidate_paths=candidate_paths,
            save_path="factory_method_grid.png"
        )
    else:
        print("\n未找到可行路径，可减少障碍物数量/密度！")