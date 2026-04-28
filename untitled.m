clear
format short

function angle = normalize_angle(angle)
    % 将角度规范化到 [-pi, pi] 区间（使用循环加减 2*pi）
    while angle > pi
        angle = angle - 2*pi;
    end
    while angle < -pi
        angle = angle + 2*pi;
    end
end

% 初始化存储所有解的矩阵
solutions = [];

% ========= 1. 定义机器人参数（MDH，无 offset）=========
alpha = [0,     -pi/2, 0,    -pi/2,   pi/2,  -pi/2];
a     = [0,     0,     2,    0.5,     0,     0];
d     = [0,     0,     0.4,  0.8,     0,     0];
test  = [-1.77, 2.45, 1.25, 0.40, -2.6, -0.34];

% 建立机械臂模型（保持 MDH）
L(1) = Link('alpha', alpha(1), 'a', a(1), 'd', d(1), 'modified');
L(1).qlim = [-pi, pi];
L(2) = Link('alpha', alpha(2), 'a', a(2), 'd', d(2), 'modified');
L(2).qlim = [-pi, pi];
L(3) = Link('alpha', alpha(3), 'a', a(3), 'd', d(3), 'modified');
L(3).qlim = [-pi, pi];
L(4) = Link('alpha', alpha(4), 'a', a(4), 'd', d(4), 'modified');
L(4).qlim = [-pi, pi];
L(5) = Link('alpha', alpha(5), 'a', a(5), 'd', d(5), 'modified');
L(5).qlim = [-pi, pi];
L(6) = Link('alpha', alpha(6), 'a', a(6), 'd', d(6), 'modified');
L(6).qlim = [-pi, pi];
robot = SerialLink(L, 'name', 'engineer');

% 显示测试位姿
robot.teach(test);
%robot.display();

% ========= 2. 正解得到末端目标位姿和T06 =========
p = robot.fkine(test);
x_in = p.t(1);
y_in = p.t(2);
z_in = p.t(3);
r_in_2 = x_in^2 + y_in^2 + z_in^2;
% 旋转矩阵第一列(即x轴)的分量
r11 = p.n(1);
r21 = p.n(2);
r31 = p.n(3);
% 旋转矩阵第一列(即y轴)的分量
r12 = p.o(1);
r22 = p.o(2);
r32 = p.o(3);
% 旋转矩阵第一列(即z轴)的分量
r13 = p.a(1);
r23 = p.a(2);
r33 = p.a(3);

T06 = [r11 r12 r13 x_in;r21 r22 r23 y_in;r31 r32 r33 z_in;0 0 0 1];

% ========= 3. 逆解算 =========

a2 = a(3);
a3 = a(4);

d3 = d(3);
d4 = d(4);

temp1 = sqrt(x_in^2 + y_in^2 - d3^2);
K = (r_in_2 - a2^2 - a3^2 - d3^2 - d4^2) / (2*a2);
temp2 = sqrt(a3^2 + d4^2 - K^2);

for i = 1:2
    % 解出theta1并规范角度
    if i == 1
        theta1 = atan2(y_in, x_in) - atan2(d3, temp1);
    else
        theta1 = atan2(y_in, x_in) - atan2(d3, -temp1);
    end
    
    % 解出theta3并规范角度
    for j = 1:2
        if j == 1
            theta3 = atan2(a3, d4) - atan2(K, temp2);
        else
            theta3 = atan2(a3, d4) - atan2(K, -temp2);
        end
        
        c1 = cos(theta1);  s1 = sin(theta1);
        c3 = cos(theta3);  s3 = sin(theta3);

        % 解theta2(纯几何法) 经过我的永不放弃的努力,我最终还是放弃用 机器人学导论原书第4版(（美）约翰·克雷格)的公式了,这段是ai给的目前测试起来没问题
        X = a2 + a3*c3 - d4*s3;
        Y = a3*s3 + d4*c3;
        u = c1*x_in + s1*y_in;
        w = z_in;
        det = u^2 + w^2;
        c2 = (u*X - w*Y) / det;
        s2 = (-u*Y - w*X) / det;
        theta2 = atan2(s2, c2);
        
        % 前三个绝对没问题



        % 解theta5
        s23 = sin(theta2 + theta3);  c23 = cos(theta2 + theta3);
        c5  = r13*(-c1*s23) + r23*(-s1*s23) + r33*(-c23);
        s5  = sqrt(1 - c5^2);
        %s5^2 + c5^2
        for k = 1:2
            if k == 1
                s5 = s5;
            elseif k == 2
                s5 = -s5;
            end
                theta5 = atan2(s5, c5);
        
         % 解theta4
         num = -r13*s1 + r23*c1;                                      
         den = r13*c1*c23 + r23*s1*c23 - r33*s23;   
         
         % 关键：除以 s5 剥离耦合符号，使 k=1 和 k=2 时 theta4 自动跳变 ±pi
         if abs(s5) > 1e-6
             s4 = num / s5;
             c4 = -den / s5;
         else
             s4 = 0; c4 = 1; % 腕部奇异保护 (s5≈0 时固定 theta4)
         end
         theta4 = atan2(s4, c4);

         % 解theta6
         num_s6 = -r11*(c1*c23*s4 - s1*c4) - r21*(s1*c23*s4 + c1*c4) + r31*(s23*s4);
         num_c6 = r11*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - r31*(s23*c4*c5 + c23*s5);
         
         % 关键：除以 s5 剥离耦合符号，使 k=1/2 时 theta6 自动跳变 ±pi
         if abs(s5) > 1e-6
             s6_val = num_s6 / s5;
             c6_val = num_c6 / s5;
         else
             s6_val = 0; c6_val = 1; % 腕部奇异保护
         end

         theta6 = atan2(s6_val, c6_val);
         if k == 2 && abs(s5) > 1e-6
             theta6 = theta6 + pi;
         end
         
         % 规范化并在显示在命令行窗口
         theta1 = normalize_angle(theta1);
         theta2 = normalize_angle(theta2);
         theta3 = normalize_angle(theta3);
         theta4 = normalize_angle(theta4);
         theta5 = normalize_angle(theta5);
         theta6 = normalize_angle(theta6);

         % 保存这组解
         solutions = [solutions; theta1, theta2, theta3, theta4, theta5, theta6];
        end
    end
end

% 展示六个角度的八组解
solutions

% ========= 4. 显示所有8组解（每个解一个独立的 teach 窗口）=========
for idx = 1:size(solutions, 1)
    figure('Name', sprintf('解 %d', idx), 'NumberTitle', 'off');
    robot.teach(solutions(idx, :));
end