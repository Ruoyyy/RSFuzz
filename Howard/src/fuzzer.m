function attack_pos = fuzzer(AttackDrone, drones,obstacles, end_loc, dt, drones_vel)
    % 候选点数量
    num_points = 20;

    % 1) 影响判定：收集可被攻击机影响到的无人机集合 S
    S = [];
    for i = 1:length(drones)
        d = norm(AttackDrone.pos - drones(i).pos);
        if d <= AttackDrone.radius + drones(i).radius
            S = [S, i];
        end
    end

    % 如果 S 为空，则退化为原逻辑：对所有无人机采样
    if isempty(S)
        S = 1:length(drones);
    end

    % 2) 在集合 S 内基于 Katz 中心性选择“最脆弱无人机”
    target_indices = S;  % 默认全部；若 Python 可用，则筛选
    try
        % 确保 Python 能找到 katz.py
        this_dir = fileparts(mfilename('fullpath'));
        if count(py.sys.path, this_dir) == 0
            insert(py.sys.path, int32(0), this_dir);
        end

        % 构造 Python 端的无人机二维位置列表（只取 x,y）
        py_drone_list = py.list();
        for k = 1:length(S)
            idx = S(k);
            x = drones(idx).pos(1);
            y = drones(idx).pos(2);
            py_drone_list.append(py.tuple({x, y}));
        end

        % 设置内部连边阈值：1.5 * 感知半径（假设各无人机半径一致）
        max_influence_distance = 1.5 * drones(S(1)).radius;

        % 计算 Katz 中心性并找到关键节点（返回局部索引，0-based）
        py_mod = py.importlib.import_module('katz');
        py_graph = py_mod.Graph(py_drone_list, max_influence_distance);
        py_vul = py_graph.vulnerable_nodes; % Python list
        vuln_list = cell(py.list(py_vul));
        % 选第一个关键节点（若有多个，任选其一）
        if ~isempty(vuln_list)
            local_idx = double(vuln_list{1}) + 1; % 转为 MATLAB 1-based
            target_indices = S(local_idx);
        end
    catch
        % Python 不可用或调用失败时，维持 target_indices = S（不降级报错）
    end

    % 3) 仅围绕目标无人机生成候选点
    candidate_points = [];
    if numel(target_indices) == 1
        i = target_indices(1);
        points = random(AttackDrone.pos', AttackDrone.radius', drones(i).pos', drones(i).radius', num_points);
        for j = 1:length(points)
            candidate_points = [candidate_points, points(:,j)];
        end
    else
        % 若未能筛出单一目标，则对集合内全部无人机生成候选点
        for ii = 1:length(target_indices)
            i = target_indices(ii);
            points = random(AttackDrone.pos', AttackDrone.radius', drones(i).pos', drones(i).radius', num_points);
            for j = 1:length(points)
                candidate_points = [candidate_points, points(:,j)];
            end
        end
    end

    % 4) 计算每个候选点的鲁棒性并选择最差者
    robustness_values = zeros(1, size(candidate_points, 2));
    for i = 1:size(candidate_points, 2)
        % 假设AttackDrone已经到达候选点
        old_pos = AttackDrone.pos;
        AttackDrone.pos = candidate_points(:,i);
        robustness_values(i) = Robustness(AttackDrone, drones, obstacles, end_loc, dt, drones_vel);
        AttackDrone.pos = old_pos; % 还原
    end

    [~, min_index] = min(robustness_values);
    attack_pos = candidate_points(:, min_index);
end
