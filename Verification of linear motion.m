clear; clc;
format short;

%% ================= 工具函数 =================
function angle = normalize_angle(angle)
    while angle > pi
        angle = angle - 2*pi;
    end
    while angle < -pi
        angle = angle + 2*pi;
    end
end

%% ================= 1. 机器人建模 (MDH) =================
alpha = [0,     -pi/2, 0,    -pi/2,   pi/2,  -pi/2];
a     = [0,     0,     2,    -0.5,     0,     0];
d     = [0,     0,    0.7,  0.8,     0,     0];
test_ = [-91, 180, -61, 90, -79, -67];          % 测试关节角(度)
test  = deg2rad(test_);

L(1) = Link('alpha', alpha(1), 'a', a(1), 'd', d(1), 'modified'); L(1).qlim = [-pi, pi];
L(2) = Link('alpha', alpha(2), 'a', a(2), 'd', d(2), 'modified'); L(2).qlim = [-pi, pi];
L(3) = Link('alpha', alpha(3), 'a', a(3), 'd', d(3), 'modified'); L(3).qlim = [-pi, pi];
L(4) = Link('alpha', alpha(4), 'a', a(4), 'd', d(4), 'modified'); L(4).qlim = [-pi, pi];
L(5) = Link('alpha', alpha(5), 'a', a(5), 'd', d(5), 'modified'); L(5).qlim = [-pi, pi];
L(6) = Link('alpha', alpha(6), 'a', a(6), 'd', d(6), 'modified'); L(6).qlim = [-pi, pi];
robot = SerialLink(L, 'name', 'engineer');

% 连杆常数
a2 = a(3); a3 = a(4);
d3 = d(3); d4 = d(4);

%% ================= 2. 逆解函数（封装的之前的算法） =================
function solutions = ikine_8solutions(T06, a2, a3, d3, d4)
    % 提取位置与姿态
    x_in = T06(1,4); y_in = T06(2,4); z_in = T06(3,4);
    r11 = T06(1,1); r21 = T06(2,1); r31 = T06(3,1);
    r12 = T06(1,2); r22 = T06(2,2); r32 = T06(3,2);
    r13 = T06(1,3); r23 = T06(2,3); r33 = T06(3,3);

    r_in_2 = x_in^2 + y_in^2 + z_in^2;
    temp1 = sqrt(x_in^2 + y_in^2 - d3^2);
    K = (r_in_2 - a2^2 - a3^2 - d3^2 - d4^2) / (2*a2);
    temp2 = sqrt(a3^2 + d4^2 - K^2);

    solutions = [];  % 存储 8 组解
    for i = 1:2
        if i == 1
            theta1 = atan2(y_in, x_in) - atan2(d3, temp1);
        else
            theta1 = atan2(y_in, x_in) - atan2(d3, -temp1);
        end

        for j = 1:2
            if j == 1
                theta3 = atan2(a3, d4) - atan2(K, temp2);
            else
                theta3 = atan2(a3, d4) - atan2(K, -temp2);
            end

            c1 = cos(theta1); s1 = sin(theta1);
            c3 = cos(theta3); s3 = sin(theta3);

            % theta2
            X = a2 + a3*c3 - d4*s3;
            Y = a3*s3 + d4*c3;
            u = c1*x_in + s1*y_in;
            w = z_in;
            det = u^2 + w^2;
            c2 = (u*X - w*Y) / det;
            s2 = (-u*Y - w*X) / det;
            theta2 = atan2(s2, c2);

            % theta5
            c23 = cos(theta2+theta3); s23 = sin(theta2+theta3);
            c5 = r13*(-c1*s23) + r23*(-s1*s23) + r33*(-c23);
            s5_abs = sqrt(1 - c5^2);

            for k = 1:2
                if k == 1
                    s5 = s5_abs;
                else
                    s5 = -s5_abs;
                end
                theta5 = atan2(s5, c5);

                % theta4
                num4 = -r13*s1 + r23*c1;
                den4 = r13*c1*c23 + r23*s1*c23 - r33*s23;
                if abs(s5) > 1e-6
                    s4 = num4 / s5;
                    c4 = -den4 / s5;
                else
                    s4 = 0; c4 = 1;
                end
                theta4 = atan2(s4, c4);

                % theta6
                num_s6 = -r11*(c1*c23*s4 - s1*c4) - r21*(s1*c23*s4 + c1*c4) + r31*(s23*s4);
                num_c6 = r11*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) ...
                       + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) ...
                       - r31*(s23*c4*c5 + c23*s5);
                if abs(s5) > 1e-6
                    s6_val = num_s6 / s5;
                    c6_val = num_c6 / s5;
                else
                    s6_val = 0; c6_val = 1;
                end
                theta6 = atan2(s6_val, c6_val);
                if k == 2 && abs(s5) > 1e-6
                    theta6 = theta6 + pi;
                end

                % 规范化并保存
                sol = [normalize_angle(theta1), normalize_angle(theta2), ...
                       normalize_angle(theta3), normalize_angle(theta4), ...
                       normalize_angle(theta5), normalize_angle(theta6)];
                solutions = [solutions; sol];
            end
        end
    end
end

%% ================= 3. 直线运动演示 =================
% 起点：使用测试关节角的当前位姿
T_start = robot.fkine(test);
% 目标位姿设定(通过移动和旋转起点得到)
% 定义平移和旋转的4x4矩阵
T_trans = [1 0 0 1.3; 0 1 0 1.2; 0 0 1 1.4; 0 0 0 1.1];

theta_z = pi/6;
Rz = [cos(theta_z), -sin(theta_z), 0, 0;
      sin(theta_z),  cos(theta_z), 0, 0;
                 0,             0, 1, 0;
                 0,             0, 0, 1];

theta_x = -pi/12;
Rx = [1,           0,            0, 0;
      0, cos(theta_x), -sin(theta_x), 0;
      0, sin(theta_x),  cos(theta_x), 0;
      0,           0,            0, 1];

theta_y = pi/12;
Ry = [ cos(theta_y), 0, sin(theta_y), 0;
                 0, 1,           0, 0;
      -sin(theta_y), 0, cos(theta_y), 0;
                 0, 0,           0, 1];

% 组合为一次复合变换（先平移，再绕定轴Z、X、Y旋转）
T_move = T_trans * Rz * Rx * Ry;

T_start_num = T_start.T; % 数据类型转换,SE3换成普通的矩阵(不然会有语法错误)
% 目标位姿
T_goal = T_start_num * T_move;

% 笛卡尔轨迹生成（位置线性插值 + 姿态球面线性插值）
steps = 100;
T_goal = SE3(T_goal); % 数据类型转换,普通矩阵换成SE3(不然会有语法错误)

% 插值,对轨迹中的移动部分进行线性插值,对于旋转部分进行先化为四元数再进行球面线性插值再转回旋转矩阵和移动部分存一起得到转换矩阵T
% 但对于旋转部分的插值,个人其实倾向于直接线性插值(这里不是),简单算力需求小,而且最后轨迹也都是直线
% 这里插值steps次,
traj_CT = ctraj(T_start, T_goal, steps);

% 逐点逆解并选择最优组
q_start = test;                  % 当前关节角作为参
q_traj = zeros(steps, 6);
q_prev = q_start;
for idx = 1:steps
    Tk = traj_CT(idx).T;          % 得到 4×4 double 矩阵
    sols = ikine_8solutions(Tk, a2, a3, d3, d4);
    % 欧几里得距离,选出最小关节转动的绝对值的和为最优解(虽然其实还是有突变出现可能,但那于算法部分无关,应该看你选的测试选用初始位姿和目标位姿)
    diff = sols - q_prev;
    dist = vecnorm(diff, 2, 2);  % 每行的2范数
    [~, minIdx] = min(dist);
    q_k = sols(minIdx, :);
    q_traj(idx, :) = q_k;
    q_prev = q_k;
end

% 计算所有末端位置
p_end = zeros(steps, 3);
for idx = 1:steps
    Tk = robot.fkine(q_traj(idx,:));
    p_end(idx,:) = Tk.t';
end

% 逐帧显示
figure('Name','直线运动验证','NumberTitle','off');
robot.teach(q_traj(1,:));          % 创建 teach 界面，显示初始位姿
hold on;

% 预先绘制完整的轨迹线 这段ai写的,仅是用来可视化结果
h_traj = plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 2);
hold off;
% 更新初始轨迹点（第一个点）
set(h_traj, 'XData', p_end(1,1), 'YData', p_end(1,2), 'ZData', p_end(1,3));

for idx = 2:steps
    robot.animate(q_traj(idx,:));   % 仅更新机器人模型，不重建 GUI
    % 更新轨迹线数据（动态延长）
    set(h_traj, 'XData', p_end(1:idx,1), ...
                'YData', p_end(1:idx,2), ...
                'ZData', p_end(1:idx,3));
    drawnow;
    pause(0);                       % 控制播放速度，可调整(0最快)
end