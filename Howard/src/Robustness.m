function avg_robustness = Robustness(AttackDrone, drones, obstacles, end_loc, dt, drones_vel)
    drones = [drones, AttackDrone];
    n = length(drones);  % 集群总无人机数量

    % 2. 初始化 R 值数组
    R_values = zeros(1, n);

    % 3. 计算每个无人机的鲁棒性 R_j
    for j = 1:n
        rob_1 = robust1_function(drones, obstacles, end_loc);
        rob_2 = robust2_function(drones, obstacles, end_loc, dt);
        rob_3 = robust3_function(drones, drones_vel, obstacles, end_loc, dt);
        rob_4 = robust4_function(drones, obstacles, end_loc, dt);
        rob_5 = robust5_function(drones, obstacles, end_loc, dt);

        % 归一化并计算 R_j
        rob_vector = [rob_1, rob_2, rob_3, rob_4, rob_5];
        rob_max = max(rob_vector);

        if rob_max == 0
            R_values(j) = 0;  % 如果最大鲁棒性为0，则R_j为0
        else
            R_values(j) = sum(rob_vector / rob_max);  % 计算 R_j
        end
    end

    % 4. 计算平均鲁棒性 \mathbb{R}
    avg_robustness = mean(R_values);  % 对所有无人机的 R_j 求平均
end

function robustness = robust1_function(drones, obstacles, end_loc)
    d_safe = 5;
    min_distance = inf;
    for i = 1:length(drones)-1
        new_pos = drones(i).pos + GradientDescentUpdate(drones(i).pos, i, drones, obstacles, end_loc);
        for j = 1:length(obstacles)
            distance = norm(new_pos - obstacles(j).pos);
            min_distance = min(min_distance, distance);
        end
    end
    robustness = min_distance - d_safe;
end

function robustness = robust2_function(drones, obstacles, end_loc, dt)
    v_max = 10;
    rob_values = [];
    for i = 1:length(drones)-1
        pos_now = drones(i).pos;
        pos_next = pos_now + GradientDescentUpdate(pos_now, i, drones, obstacles, end_loc);
        v = norm(pos_next - pos_now) / dt;
        rob_values = [rob_values, min(v - 0, v_max - v)];
    end
    robustness = min(rob_values);
end

function robustness = robust3_function(drones, drones_vel, obstacles, end_loc, dt)
    a_max = 2.5;
    rob_values = [];
    for i = 1:length(drones)-1
        pos_now = drones(i).pos;
        pos_next = pos_now + GradientDescentUpdate(pos_now, i, drones, obstacles, end_loc);
        v_now = norm(pos_next - pos_now) / dt;
        a = (v_now - drones_vel(i)) / dt;
        rob_values = [rob_values, a_max - abs(a)];
    end
    robustness = min(rob_values);
end

function robustness = robust4_function(drones, obstacles, end_loc, dt)
    d_min = 3;
    rob_values = [];
    updated_positions = cell(1, length(drones)-1);
    for i = 1:length(drones)-1
        updated_positions{i} = drones(i).pos + GradientDescentUpdate(drones(i).pos, i, drones, obstacles, end_loc);
    end
    for i = 1:length(updated_positions)
        for j = i+1:length(updated_positions)
            d = norm(updated_positions{i} - updated_positions{j});
            rob_values = [rob_values, d - d_min];
        end
    end
    robustness = min(rob_values);
end

function robustness = robust5_function(drones, obstacles, end_loc, dt)
    rob_values = [];
    for i = 1:length(drones)-1
        pos_now = drones(i).pos;
        pos_next = pos_now + GradientDescentUpdate(pos_now, i, drones, obstacles, end_loc);
        dist_now = norm(pos_now - end_loc);
        dist_future = norm(pos_next - end_loc);
        rob_values = [rob_values, dist_now - dist_future];
    end
    robustness = min(rob_values);
end
