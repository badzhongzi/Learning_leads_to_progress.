clear; clc;
format short;

%% ================= 机器人建模 (MDH) =================
% 连杆参数
alpha = [0,     -pi/2, 0,    -pi/2,   pi/2,  -pi/2];
a     = [0,     0,     2,    -0.5,     0,     0];
d     = [0,     0,    0.7,  0.8,     0,     0];

% 创建Link对象，并设置关节限位（全部 ±pi）
L(1) = Link('alpha', alpha(1), 'a', a(1), 'd', d(1), 'modified'); L(1).qlim = [-pi, pi];
L(2) = Link('alpha', alpha(2), 'a', a(2), 'd', d(2), 'modified'); L(2).qlim = [-pi, pi];
L(3) = Link('alpha', alpha(3), 'a', a(3), 'd', d(3), 'modified'); L(3).qlim = [-pi, pi];
L(4) = Link('alpha', alpha(4), 'a', a(4), 'd', d(4), 'modified'); L(4).qlim = [-pi, pi];
L(5) = Link('alpha', alpha(5), 'a', a(5), 'd', d(5), 'modified'); L(5).qlim = [-pi, pi];
L(6) = Link('alpha', alpha(6), 'a', a(6), 'd', d(6), 'modified'); L(6).qlim = [-pi, pi];

robot = SerialLink(L, 'name', 'engineer');

%% ================= 蒙特卡洛法工作空间 =================
numSamples = 50000;               % 采样点数（可调整）
points = zeros(numSamples, 3);    % 存储末端笛卡尔坐标

fprintf('正在采样 %d 个随机关节角组合,等待时间会有点长...\n', numSamples);
for i = 1:numSamples
    % 在每个关节的限位内生成随机角度
    q = zeros(1, robot.n);
    for j = 1:robot.n
        q(j) = L(j).qlim(1) + (L(j).qlim(2) - L(j).qlim(1)) * rand();
    end
    % 正运动学，得到末端位姿
    T = robot.fkine(q);
    % 提取位置 (x, y, z) 并存入
    points(i, :) = T.t';   % T.t 是 3×1 列向量，转置为行向量
end

%% ================= 绘制工作空间点云 ================= (这段依旧是ai写的,问就是我不会可视化)
% 创建图形窗口，调整大小以便显示
figure('Name', '机械臂工作空间 (蒙特卡洛法)', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

% 使用 tiledlayout 实现灵活布局
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% 左侧：三维点云图
ax3D = nexttile;
scatter3(points(:,1), points(:,2), points(:,3), 2, points(:,3), '.');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('三维工作空间');
axis equal; grid on; view(3);
colormap(ax3D, jet);          % 使用jet颜色映射
c = colorbar(ax3D);
c.Label.String = 'Z 坐标 (m)';
caxis(ax3D, [min(points(:,3)), max(points(:,3))]);

% 右侧：三个投影子图（并排垂直排列）
% 方法：在右侧再创建一个垂直的 tile layout
nexttile;  % 占位，实际使用嵌套布局

% 更清晰的方式：直接创建三个 axes 在右侧区域
% 获取右侧区域位置，手动创建 axes
% 或者使用 tiledlayout 嵌套，此处为简便，手动计算位置
drawnow;
rightPos = [0.55, 0.1, 0.4, 0.8];  % [left, bottom, width, height]
% 三个子图的相对高度（等分）
h_sub = rightPos(4) / 3;
y_positions = [rightPos(2) + 2*h_sub, rightPos(2) + h_sub, rightPos(2)];

% 共享颜色映射和颜色条（使用相同的颜色数据、caxis范围）
c_min = min(points(:,3));
c_max = max(points(:,3));

% 1. XOY 投影 (x, y)
axXOY = axes('Position', [rightPos(1), y_positions(1), rightPos(3), h_sub]);
scatter(points(:,1), points(:,2), 2, points(:,3), '.');
xlabel('X (m)'); ylabel('Y (m)');
title('XOY 平面投影');
axis equal; grid on;
colormap(axXOY, jet);
caxis([c_min, c_max]);

% 2. XOZ 投影 (x, z)
axXOZ = axes('Position', [rightPos(1), y_positions(2), rightPos(3), h_sub]);
scatter(points(:,1), points(:,3), 2, points(:,3), '.');
xlabel('X (m)'); ylabel('Z (m)');
title('XOZ 平面投影');
axis equal; grid on;
colormap(axXOZ, jet);
caxis([c_min, c_max]);

% 3. YOZ 投影 (y, z)
axYOZ = axes('Position', [rightPos(1), y_positions(3), rightPos(3), h_sub]);
scatter(points(:,2), points(:,3), 2, points(:,3), '.');
xlabel('Y (m)'); ylabel('Z (m)');
title('YOZ 平面投影');
axis equal; grid on;
colormap(axYOZ, jet);
caxis([c_min, c_max]);

% 添加一个总标题（可选）
sgtitle(sprintf('机械臂工作空间 (采样数 = %d) - 颜色表示末端 Z 坐标', numSamples));