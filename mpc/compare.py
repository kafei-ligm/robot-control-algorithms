import casadi as ca
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.rcParams['font.family'] = 'SimHei'

# ===== 公共系统参数 =====
M = 1.0; B = 2.0; dt = 0.01
sim_time = 3.0
steps = int(sim_time / dt)
target_pos = 1.0
t = np.linspace(0, sim_time, steps+1)

# ===== PID仿真 =====
Kp, Ki, Kd = 50, 10, 20
integral_limit, output_limit = 50, 100

pos_pid = [0.0]; vel_pid = [0.0]; u_pid = [0.0]
integral = 0; last_error = 0

for i in range(steps):
    error = target_pos - pos_pid[-1]
    integral = np.clip(integral + error * dt, -integral_limit, integral_limit)
    derivative = (error - last_error) / dt
    output = np.clip(Kp*error + Ki*integral + Kd*derivative, -output_limit, output_limit)
    acc = (output - B*vel_pid[-1]) / M
    vel_pid.append(vel_pid[-1] + acc * dt)
    pos_pid.append(pos_pid[-1] + vel_pid[-2] * dt)
    u_pid.append(output)
    last_error = error

# ===== MPC仿真 =====
A = np.array([[1, dt], [0, 1 - B/M*dt]])
B_ctrl = np.array([[0], [dt/M]])
N = 20
Q = np.diag([100.0, 1.0]); R = np.array([[0.1]])

opti = ca.Opti()
X = opti.variable(2, N+1); U = opti.variable(1, N)
x0 = opti.parameter(2); x_ref = opti.parameter(2)

cost = 0
for k in range(N):
    e = X[:, k] - x_ref
    cost += ca.mtimes(e.T, ca.mtimes(Q, e)) + ca.mtimes(U[:, k].T, ca.mtimes(R, U[:, k]))
cost += ca.mtimes((X[:, N]-x_ref).T, ca.mtimes(Q*10, (X[:, N]-x_ref)))
opti.minimize(cost)
opti.subject_to(X[:, 0] == x0)
for k in range(N):
    opti.subject_to(X[:, k+1] == ca.mtimes(A, X[:, k]) + ca.mtimes(B_ctrl, U[:, k]))
    opti.subject_to(opti.bounded(-100, U[:, k], 100))
opti.solver('ipopt', {'print_time': 0}, {'print_level': 0})

x_cur = np.array([0.0, 0.0])
pos_mpc = [0.0]; u_mpc = [0.0]

print("运行MPC...")
for i in range(steps):
    opti.set_value(x0, x_cur)
    opti.set_value(x_ref, np.array([target_pos, 0.0]))
    sol = opti.solve()
    u_apply = sol.value(U[:, 0])
    x_cur = A @ x_cur + B_ctrl.flatten() * u_apply
    pos_mpc.append(x_cur[0])
    u_mpc.append(u_apply)

print("完成！画图中...")

# ===== 对比画图 =====
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))

ax1.plot(t, pos_pid, 'b-', linewidth=2, label=f'PID (Kp={Kp}, Ki={Ki}, Kd={Kd})')
ax1.plot(t, pos_mpc, 'r-', linewidth=2, label=f'MPC (N={N}, Q=100, R=0.1)')
ax1.axhline(y=1.0, color='k', linestyle='--', linewidth=1.5, label='目标位置')
ax1.set_ylabel('位置 (rad)', fontsize=12)
ax1.set_title('PID vs MPC 关节位置控制对比', fontsize=14)
ax1.legend(fontsize=11); ax1.grid(True)

ax2.plot(t, u_pid, 'b-', linewidth=2, label='PID 力矩')
ax2.plot(t, u_mpc, 'r-', linewidth=2, label='MPC 力矩')
ax2.set_ylabel('控制力矩 (N·m)', fontsize=12)
ax2.set_xlabel('时间 (s)', fontsize=12)
ax2.set_title('控制力矩对比（体现MPC的平滑性）', fontsize=14)
ax2.legend(fontsize=11); ax2.grid(True)

plt.tight_layout()
plt.savefig('pid_vs_mpc.png', dpi=150)
plt.show()
print("图片已保存为 pid_vs_mpc.png")
