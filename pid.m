% ===== 单关节PID位置控制仿真 =====
% 系统：二阶系统模拟机械臂关节（质量-阻尼-弹簧）
% 目标：从0rad运动到1.0rad

%% 参数设置
dt = 0.01;          % 控制周期 10ms
t = 0:dt:3;         % 仿真3秒
target = 1.0;       % 目标角度 1.0 rad

% PID参数（先用这组，跑通后再调）
Kp = 5;
Ki = 0;
Kd = 5;

% 系统物理参数（模拟机械臂关节）
M = 1.0;    % 等效质量 kg
B = 2.0;    % 阻尼系数
K = 0.0;    % 弹簧系数（关节无弹性，设0）

%% 初始化
pos = zeros(size(t));   % 位置
vel = zeros(size(t));   % 速度
u   = zeros(size(t));   % 控制输出

integral   = 0;
last_error = 0;
integral_limit = 50;
output_limit   = 100;

%% 仿真主循环
for i = 2:length(t)
    error = target - pos(i-1);

    % 积分（带限幅）
    integral = integral + error * dt;
    integral = max(min(integral, integral_limit), -integral_limit);

    % 微分
    derivative = (error - last_error) / dt;

    % PID输出（带限幅）
    output = Kp*error + Ki*integral + Kd*derivative;
    output = max(min(output, output_limit), -output_limit);
    u(i) = output;

    % 系统动力学：M*a = u - B*v - K*x
    acc = (output - B*vel(i-1) - K*pos(i-1)) / M;
    vel(i) = vel(i-1) + acc * dt;
    pos(i) = pos(i-1) + vel(i-1) * dt;

    last_error = error;
end

%% 画图
figure('Position', [100 100 900 600]);

subplot(3,1,1);
plot(t, pos, 'b-', 'LineWidth', 2); hold on;
plot(t, target*ones(size(t)), 'r--', 'LineWidth', 1.5);
ylabel('位置 (rad)'); title('关节位置响应');
legend('实际位置','目标位置'); grid on;

subplot(3,1,2);
plot(t, vel, 'g-', 'LineWidth', 2);
ylabel('速度 (rad/s)'); title('关节速度');  grid on;

subplot(3,1,3);
plot(t, u, 'm-', 'LineWidth', 2);
ylabel('控制输出 (N·m)'); xlabel('时间 (s)');
title('PID控制力矩'); grid on;
