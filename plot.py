import matplotlib.pyplot as plt
import colorsys
from env import Node, GridMap
from typing import List, Tuple, Optional, Union

def print_grid_map(grid_map: GridMap, start: Tuple[int, int], goal: Tuple[int, int]):
    """打印地图，直观确认障碍物位置"""
    print("=" * 60)
    print(f"地图尺寸：{grid_map.width}×{grid_map.height}")
    print(f"起点：{start} | 终点：{goal}")
    print("栅格地图（0=可行，1=障碍物，S=起点，G=终点）：")
    for y in range(grid_map.height):
        row = []
        for x in range(grid_map.width):
            if (x, y) == start:
                row.append("S")
            elif (x, y) == goal:
                row.append("G")
            elif not grid_map.grid[y][x]:
                row.append("1")
            else:
                row.append("0")
        print(" ".join(row))
    print("=" * 60)


def visualize_grid_and_paths(
    grid_map: GridMap, 
    start: Tuple[int, int], 
    goal: Tuple[int, int],
    candidate_paths: List[List[Tuple[int, int]]], 
    save_path: Optional[str] = None,
    colormap: str = 'tab10',
    figsize: Tuple[float, float] = (20, 20),  # 增大画布尺寸（默认12×8英寸）
    dpi: int = 600  # 保持高清分辨率，可根据需求调至400/600
):
    # 1. 创建更大的画布
    plt.figure(figsize=figsize)  # 关键：用传入的figsize（默认12×8）

    # 2. 绘制栅格（保持原逻辑，栅格大小可适当调大）
    for y in range(grid_map.height):
        for x in range(grid_map.width):
            if not grid_map.grid[y][x]:
                # 障碍物：黑色方块，s=200（原s=100，调大更清晰）
                plt.scatter(x, y, c='black', s=200, marker='s', edgecolors='gray')
            else:
                # 可行区域：白色方块，s=200
                plt.scatter(x, y, c='white', s=200, marker='s', edgecolors='gray')

    # 3. 绘制起点终点（调大尺寸，更醒目）
    plt.scatter(start[0], start[1], c='red', s=400, marker='*', label='Start', edgecolors='darkred')
    plt.scatter(goal[0], goal[1], c='green', s=400, marker='P', label='Goal', edgecolors='darkgreen')

    # 4. 绘制路径（线宽调粗，颜色自动生成）


    num_paths = len(candidate_paths)
    for i, path in enumerate(candidate_paths):
        # HSV 颜色：色相均匀分布（0~1），饱和度和亮度固定
        hue = i / num_paths
        r, g, b = colorsys.hsv_to_rgb(hue, 0.8, 0.9)
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        plt.plot(path_x, path_y, color=(r, g, b), linewidth=3, label=f'Path {i+1}', alpha=0.85)

    # 5. 配置（确保元素不拥挤）
    plt.xlim(-0.8, grid_map.width - 0.2)  # 左右留少量空白，避免栅格贴边
    plt.ylim(-0.8, grid_map.height - 0.2)
    plt.xticks(range(grid_map.width), fontsize=12)  # 增大坐标轴字体
    plt.yticks(range(grid_map.height), fontsize=12)
    plt.grid(True, linewidth=1.5)  # 网格线加粗，更清晰
    plt.legend(loc='upper right', fontsize=11, frameon=True, shadow=True)  # 图例字体调大，加阴影
    plt.title('Focal Search Candidate Paths', fontsize=14, pad=20)  # 标题字体调大，增加边距

    # 6. 保存图片（用指定的dpi，bbox_inches='tight'避免内容被截断）
    if save_path:
        plt.savefig(
            save_path, 
            dpi=dpi,  # 高清分辨率
            bbox_inches='tight',  # 关键：自动调整边距，确保所有元素（如图例、标题）都被保存
            facecolor='white'  # 背景设为白色，避免透明背景
        )

    plt.show()