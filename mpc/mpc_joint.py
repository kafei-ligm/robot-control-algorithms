import casadi as ca
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.rcParams['font.family'] = 'SimHei'

# ===== 系统参数（和PID用同一个二阶系统，方便对比）=====
M = 1.0   # 等效质量
B = 2.0   # 阻尼系数
dt = 0.01 # 控制周期

# 离散化状态方程 x(k+1) = A*x(k) + B_ctrl*u(k)
# 状态 x = [位置, 速度]，输入 u = 力矩
A = np.array([[1, dt],
              [-0, 1 - B/M*dt]])
B_ctrl = np.array([[0],
                   [dt/M]])

# ===== MPC参数 =====
N = 20        # 预测时域（往前看20步）
Q = np.diag([100.0, 1.0])   # 状态权重：位置误差100，速度误差1
R = np.array([[0.1]])        # 控制权重：力矩消耗

# 约束
u_max = 100.0   # 力矩上限
u_min = -100.0
x_max = np.array([3.14, 10.0])   # 位置±180度，速度±10rad/s
x_min = np.array([-3.14, -10.0])

# ===== 构建MPC优化问题（casadi）=====
opti = ca.Opti()

# 决策变量
X = opti.variable(2, N+1)   # 预测状态序列
U = opti.variable(1, N)     # 预测控制序列

# 参数：当前状态和目标状态
x0 = opti.parameter(2)
x_ref = opti.parameter(2)

# 目标函数
cost = 0
for k in range(N):
    state_error = X[:, k] - x_ref
    cost += ca.mtimes(state_error.T, ca.mtimes(Q, state_error))
    cost += ca.mtimes(U[:, k].T, ca.mtimes(R, U[:, k]))
# 终端代价
state_error_N = X[:, N] - x_ref
cost += ca.mtimes(state_error_N.T, ca.mtimes(Q*10, state_error_N))
opti.minimize(cost)

# 约束条件
opti.subject_to(X[:, 0] == x0)  # 初始状态
for k in range(N):
    # 系统动力学约束
    opti.subject_to(X[:, k+1] == ca.mtimes(A, X[:, k]) + ca.mtimes(B_ctrl, U[:, k]))
    # 输入约束
    opti.subject_to(U[:, k] <= u_max)
    opti.subject_to(U[:, k] >= u_min)
    # 状态约束
    opti.subject_to(X[:, k] <= x_max)
    opti.subject_to(X[:, k] >= x_min)

# 求解器设置（ipopt，打印关掉）
opti.solver('ipopt', {'print_time': 0}, {'print_level': 0})

# ===== 仿真主循环 =====
sim_time = 3.0
steps = int(sim_time / dt)
target = np.array([1.0, 0.0])   # 目标：位置1.0rad，速度0

x_current = np.array([0.0, 0.0])  # 初始状态
pos_hist = [x_current[0]]
vel_hist = [x_current[1]]
u_hist   = [0.0]

print("开始MPC仿真...")
for i in range(steps):
    opti.set_value(x0, x_current)
    opti.set_value(x_ref, target)

    sol = opti.solve()
    u_apply = sol.value(U[:, 0])  # 只取第一步

    # 系统动力学更新
    x_next = A @ x_current + B_ctrl.flatten() * u_apply
    x_current = x_next

    pos_hist.append(x_current[0])
    vel_hist.append(x_current[1])
    u_hist.append(u_apply)

    if i % 50 == 0:
        print(f"  t={i*dt:.2f}s, pos={x_current[0]:.4f}, u={u_apply:.2f}")

print("仿真完成！")

# ===== 画图 =====
t = np.linspace(0, sim_time, steps+1)

fig, axes = plt.subplots(3, 1, figsize=(10, 8))

axes[0].plot(t, pos_hist, 'b-', linewidth=2, label='实际位置')
axes[0].axhline(y=1.0, color='r', linestyle='--', linewidth=1.5, label='目标位置')
axes[0].set_ylabel('位置 (rad)')
axes[0].set_title('MPC关节位置控制')
axes[0].legend(); axes[0].grid(True)

axes[1].plot(t, vel_hist, 'g-', linewidth=2)
axes[1].set_ylabel('速度 (rad/s)')
axes[1].set_title('关节速度')
axes[1].grid(True)

axes[2].plot(t, u_hist, 'm-', linewidth=2)
axes[2].set_ylabel('控制力矩 (N·m)')
axes[2].set_xlabel('时间 (s)')
axes[2].set_title('MPC控制输出')
axes[2].grid(True)

plt.tight_layout()
plt.savefig('mpc_result.png', dpi=150)
plt.show()
