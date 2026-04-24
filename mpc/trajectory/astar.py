import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import heapq
matplotlib.rcParams['font.family'] = 'SimHei'

# ===== 地图设置 =====
grid = np.zeros((20, 20))
# 添加障碍物
obstacles = [(5,2),(5,3),(5,4),(5,5),(5,6),
             (10,8),(10,9),(10,10),(10,11),(10,12),
             (3,15),(4,15),(5,15),(6,15),(7,15)]
for r,c in obstacles:
    grid[r][c] = 1

start = (1, 1)
goal  = (18, 18)

# ===== A*实现 =====
def heuristic(a, b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)  # 欧几里得距离

def astar(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    visited = []

    while open_set:
        _, current = heapq.heappop(open_set)
        visited.append(current)

        if current == goal:
            # 回溯路径
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1], visited

        # 8方向邻居
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),
                        (-1,-1),(-1,1),(1,-1),(1,1)]:
            neighbor = (current[0]+dr, current[1]+dc)
            if (0 <= neighbor[0] < grid.shape[0] and
                0 <= neighbor[1] < grid.shape[1] and
                grid[neighbor] == 0):
                
                move_cost = 1.414 if abs(dr)+abs(dc)==2 else 1.0
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None, visited

path, visited = astar(grid, start, goal)
print(f"路径长度: {len(path)} 步")
print(f"搜索节点数: {len(visited)}")

# ===== 画图 =====
fig, ax = plt.subplots(1, 1, figsize=(8, 8))

# 画地图
display = np.zeros_like(grid, dtype=float)
for r,c in visited:
    display[r][c] = 0.3   # 搜索过的节点（浅色）
for r,c in path:
    display[r][c] = 0.7   # 最终路径（深色）
for r,c in obstacles:
    display[r][c] = 1.0   # 障碍物（黑色）

ax.imshow(display, cmap='Blues', origin='upper', vmin=0, vmax=1)

# 标记起点终点
ax.plot(start[1], start[0], 'gs', markersize=15, label='起点')
ax.plot(goal[1],  goal[0],  'r*', markersize=15, label='终点')

# 画路径连线
path_r = [p[0] for p in path]
path_c = [p[1] for p in path]
ax.plot(path_c, path_r, 'r-', linewidth=2, label=f'A*路径({len(path)}步)')

ax.set_title(f'A* 路径规划\n搜索节点:{len(visited)}  路径长度:{len(path)}步', fontsize=13)
ax.legend(loc='upper left')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('astar_result.png', dpi=150)
plt.show()
