% 定义两个球体的中心和半径
center1 = [0, 0, 0]; % 第一个球体的中心
center2 = [1, 0, 0]; % 第二个球体的中心
radius1 = 2; % 第一个球体的半径
radius2 = 2; % 第二个球体的半径

% 定义要生成的点的数量
numPoints = 100;

% 调用函数生成点
random_points = random(center1, radius1, center2, radius2, numPoints);

% 显示生成的点
scatter3(random_points(1, :), random_points(2, :), random_points(3, :), 'filled');
hold on;
[X, Y, Z] = sphere;
surf(radius1*X + center1(1), radius1*Y + center1(2), radius1*Z + center1(3), 'FaceAlpha', 0.2);
surf(radius2*X + center2(1), radius2*Y + center2(2), radius2*Z + center2(3), 'FaceAlpha', 0.2);
axis equal;
hold off;
