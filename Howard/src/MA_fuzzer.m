function attack_pos = MA_fuzzer(AttackDrone, drones,obstacles, end_loc, dt, drones_vel)
    num_points = 20;
    candidate_points = [];
    for i = 1:length(drones)
        % MA_fuzzing
        % 在当前无人机的radius内随机生成点
        points = random_within_radius(drones(i).pos', drones(i).radius', num_points);
        for j = 1:length(points)
            candidate_points = [candidate_points, points(:,j)];
        end
    end
    % 计算每个候选点的鲁棒性
    robustness_values = zeros(num_points, 1);
    for i = 1:num_points
        % 假设AttackDrone已经到达候选点
        AttackDrone.pos = candidate_points(:,i);
        % 计算鲁棒性
        robustness_values(i) = Robustness(AttackDrone, drones, obstacles, end_loc, dt, drones_vel);
    end
    
    % 选择鲁棒性最低的点
    [~, min_index] = min(robustness_values);
    attack_pos = candidate_points(:, min_index);
end

function points = random_within_radius(center, radius, num_points)
    % 在给定中心和半径的球体内随机生成点
    points = zeros(3, num_points);  % 初始化一个3行num_points列的矩阵，用于存储生成的点
    for i = 1:num_points
        theta = 2 * pi * rand();  % 生成一个在[0, 2*pi)范围内的随机角度theta
        phi = acos(2 * rand() - 1);  % 生成一个在[0, pi]范围内的随机角度phi
        r = radius * rand();  % 生成一个在[0, radius)范围内的随机半径r
        % 根据球坐标转换公式，计算三维坐标
        points(:,i) = center' + [r * sin(phi) * cos(theta); r * sin(phi) * sin(theta); r * cos(phi)];
    end
end

