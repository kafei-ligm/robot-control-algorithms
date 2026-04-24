import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
matplotlib.rcParams['font.family'] = 'SimHei'

# ===== 工具函数 =====
def path_length(path):
    pts = np.array(path)
    return np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))

def collision_free(p1, p2, obstacles, check_points=10):
    for t in np.linspace(0, 1, check_points):
        p = p1 + t * (p2 - p1)
        for cx, cy, r in obstacles:
            if np.hypot(p[0]-cx, p[1]-cy) < r + 0.5:
                return False
    return True

def bspline_smooth(path, obstacles, smooth_points=300):
    if len(path) < 4:
        return path
    pts = np.array(path)
    tck, _ = splprep([pts[:,0], pts[:,1]], s=2, k=3)
    u = np.linspace(0, 1, smooth_points)
    x, y = splev(u, tck)
    smooth = list(zip(x, y))
    for p in smooth:
        for cx, cy, r in obstacles:
            if np.hypot(p[0]-cx, p[1]-cy) < r:
                return path  # 穿障则返回原始路径
    return smooth

# ===== 地图设置 =====
MAP_SIZE = 100
obstacles = [
    (25, 15, 7), (20, 40, 9), (15, 70, 7),
    (45, 25, 8), (40, 55, 10), (45, 80, 7),
    (65, 20, 6), (65, 45, 9), (70, 75, 8),
    (85, 55, 6)
]
start = np.array([5.0, 5.0])
goal  = np.array([95.0, 95.0])

# ===== 标准 RRT* =====
def rrt_star(start, goal, obstacles, max_iter=1000,
             step_size=5.0, goal_radius=3.0, goal_bias=0.1, search_r=15.0):
    nodes = [start.copy()]
    parent = {0: -1}
    cost = {0: 0.0}

    for _ in range(max_iter):
        if np.random.rand() < goal_bias:
            q_rand = goal.copy()
        else:
            q_rand = np.random.uniform(0, MAP_SIZE, 2)

        dists = [np.linalg.norm(q_rand - nodes[i]) for i in range(len(nodes))]
        i_near = int(np.argmin(dists))
        q_near = nodes[i_near]

        direction = q_rand - q_near
        d = np.linalg.norm(direction)
        if d < 1e-6: continue
        q_new = q_near + (direction/d) * min(step_size, d)

        if not collision_free(q_near, q_new, obstacles): continue

        i_new = len(nodes)
        best_parent = i_near
        best_cost = cost[i_near] + np.linalg.norm(q_new - q_near)

        for i, node in enumerate(nodes):
            if np.linalg.norm(q_new - node) < search_r:
                c = cost[i] + np.linalg.norm(q_new - node)
                if c < best_cost and collision_free(node, q_new, obstacles):
                    best_cost = c
                    best_parent = i

        nodes.append(q_new)
        parent[i_new] = best_parent
        cost[i_new] = best_cost

        for i, node in enumerate(nodes[:-1]):
            if np.linalg.norm(q_new - node) < search_r:
                c = best_cost + np.linalg.norm(node - q_new)
                if c < cost[i] and collision_free(q_new, node, obstacles):
                    parent[i] = i_new
                    cost[i] = c

        if np.linalg.norm(q_new - goal) < goal_radius:
            break

    path = []
    i = len(nodes) - 1
    while i != -1:
        path.append(nodes[i])
        i = parent[i]
    path.reverse()
    return path, nodes, len(nodes)

# ===== 改进 RRT* =====
def improved_rrt_star(start, goal, obstacles, max_iter=1000,
                      step_min=2.0, step_max=8.0, goal_radius=3.0,
                      p0=0.6, p_min=0.1, search_r=15.0):
    nodes = [start.copy()]
    parent = {0: -1}
    cost = {0: 0.0}
    step_size = 5.0

    for iter_i in range(max_iter):
        # 改进1：动态目标偏置概率
        p = p0 - (iter_i / max_iter) * (p0 - p_min)
        if np.random.rand() < p:
            q_rand = goal.copy()
        else:
            q_rand = np.random.uniform(0, MAP_SIZE, 2)

        dists = [np.linalg.norm(q_rand - nodes[i]) for i in range(len(nodes))]
        i_near = int(np.argmin(dists))
        q_near = nodes[i_near]

        direction = q_rand - q_near
        d = np.linalg.norm(direction)
        if d < 1e-6: continue
        q_new = q_near + (direction/d) * min(step_size, d)

        # 改进2：自适应步长
        if not collision_free(q_near, q_new, obstacles):
            step_size = max(step_min, step_size * 0.8)
            continue
        else:
            step_size = min(step_max, step_size * 1.2)

        i_new = len(nodes)
        best_parent = i_near
        best_cost = cost[i_near] + np.linalg.norm(q_new - q_near)

        for i, node in enumerate(nodes):
            if np.linalg.norm(q_new - node) < search_r:
                c = cost[i] + np.linalg.norm(q_new - node)
                if c < best_cost and collision_free(node, q_new, obstacles):
                    best_cost = c
                    best_parent = i

        nodes.append(q_new)
        parent[i_new] = best_parent
        cost[i_new] = best_cost

        for i, node in enumerate(nodes[:-1]):
            if np.linalg.norm(q_new - node) < search_r:
                c = best_cost + np.linalg.norm(node - q_new)
                if c < cost[i] and collision_free(q_new, node, obstacles):
                    parent[i] = i_new
                    cost[i] = c

        if np.linalg.norm(q_new - goal) < goal_radius:
            break

    path = []
    i = len(nodes) - 1
    while i != -1:
        path.append(nodes[i])
        i = parent[i]
    path.reverse()
    return path, nodes, len(nodes)

# ===== 运行对比 =====
print("运行标准 RRT*...")
np.random.seed(42)
path1, nodes1, iters1 = rrt_star(start, goal, obstacles)

print("运行改进 RRT*...")
np.random.seed(42)
path2, nodes2, iters2 = improved_rrt_star(start, goal, obstacles)
path2_smooth = bspline_smooth(path2, obstacles)

len1 = path_length(path1)
len2 = path_length(path2)
len2s = path_length(path2_smooth)

print(f"\n标准RRT*:  节点数={iters1},  路径长度={len1:.1f}")
print(f"改进RRT*:  节点数={iters2},  路径长度={len2:.1f}")
print(f"B样条平滑: 路径长度={len2s:.1f}")

# ===== 画图 =====
fig, axes = plt.subplots(1, 2, figsize=(14, 7))

titles = [
    f'标准 RRT*\n节点数:{iters1}  路径长度:{len1:.1f}',
    f'改进 RRT*（动态偏置+自适应步长+B样条）\n节点数:{iters2}  平滑路径长度:{len2s:.1f}'
]

for idx, ax in enumerate(axes):
    nodes = nodes1 if idx == 0 else nodes2
    path  = path1  if idx == 0 else path2

    # 画树节点
    for node in nodes:
        ax.plot(node[0], node[1], 'o', color='lightgray', markersize=1.5, zorder=1)

    # 画障碍物
    for cx, cy, r in obstacles:
        ax.add_patch(plt.Circle((cx, cy), r, color='dimgray', zorder=2))

    # 画原始路径
    if len(path) > 1:
        px = [p[0] for p in path]
        py = [p[1] for p in path]
        color = 'steelblue' if idx == 0 else 'salmon'
        ax.plot(px, py, '-', color=color, linewidth=1.5,
                alpha=0.6, label='原始路径', zorder=3)

    # 改进版额外画B样条
    if idx == 1 and len(path2_smooth) > 1:
        sx = [p[0] for p in path2_smooth]
        sy = [p[1] for p in path2_smooth]
        ax.plot(sx, sy, '-', color='orange', linewidth=2.5,
                label='B样条平滑', zorder=4)

    ax.plot(*start, 'gs', markersize=12, label='起点', zorder=5)
    ax.plot(*goal,  'r*', markersize=14, label='终点', zorder=5)
    ax.set_xlim(0, MAP_SIZE); ax.set_ylim(0, MAP_SIZE)
    ax.set_title(titles[idx], fontsize=11)
    ax.legend(loc='upper left', fontsize=9)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('rrt_comparison.png', dpi=150)
plt.show()
print("完成！图片已保存为 rrt_comparison.png")