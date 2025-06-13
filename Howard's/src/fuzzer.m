function attack_pos = fuzzer(AttackDrone, drones,obstacles, end_loc, dt, drones_vel)
    num_points = 20;
    candidate_points = [];
    for i = 1:length(drones)
        % 计算两个球体的距离
        d = norm(AttackDrone.pos - drones(i).pos);
    
        % 检查两个球体是否相交
        if d > AttackDrone.radius + drones(i).radius
            continue; % 不相交则跳过
        end
        % 在交集范围内随机生成点
        points = random(AttackDrone.pos', AttackDrone.radius', drones(i).pos', drones(i).radius', num_points);
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
    disp(min(robustness_values))
    attack_pos = candidate_points(:, min_index);
end
