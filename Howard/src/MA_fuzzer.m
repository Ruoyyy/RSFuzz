function attack_pos = MA_fuzzer(AttackDrone, drones,obstacles, end_loc, dt, drones_vel)
    % 多智能体高级模糊（MA）：
    % 1) 用所有无人机构图，Katz 中心性找最脆弱无人机
    % 2) 仅在该目标无人机的影响范围内随机采样候选点（不考虑攻击机可达）
    % 3) 用鲁棒性最小准则选择最终攻击位

    num_points = 20;

    % 1) 全体无人机构图 → Katz 目标
    target_index = 1; % 默认回退
    try
        % 确保 Python 路径包含当前目录
        this_dir = fileparts(mfilename('fullpath'));
        if count(py.sys.path, this_dir) == 0
            insert(py.sys.path, int32(0), this_dir);
        end

        % 三维坐标列表 (x,y,z)
        py_drone_list = py.list();
        for i = 1:length(drones)
            x = drones(i).pos(1); y = drones(i).pos(2); z = drones(i).pos(3);
            py_drone_list.append(py.tuple({x, y, z}));
        end

        % 全局内部连边阈值：1.5 × 感知半径（假设一致）
        max_influence_distance = 1.5 * drones(1).radius;

        py_mod = py.importlib.import_module('katz');
        py_graph = py_mod.Graph(py_drone_list, max_influence_distance);
        py_vul = py_graph.vulnerable_nodes; % Python list of node ids
        vuln_list = cell(py.list(py_vul));
        if ~isempty(vuln_list)
            target_index = double(vuln_list{1}) + 1; % MATLAB 1-based
        end
    catch
        % 保持默认 target_index = 1
    end

    % 2) 在目标无人机影响范围（其半径）内随机生成候选点
    candidate_points = [];
    points = random_within_radius(drones(target_index).pos', drones(target_index).radius', num_points);
    for j = 1:length(points)
        candidate_points = [candidate_points, points(:,j)];
    end

    % 3) 逐点评分鲁棒性，选择最小
    robustness_values = zeros(1, size(candidate_points, 2));
    for i = 1:size(candidate_points, 2)
        old_pos = AttackDrone.pos;
        AttackDrone.pos = candidate_points(:,i);
        robustness_values(i) = Robustness(AttackDrone, drones, obstacles, end_loc, dt, drones_vel);
        AttackDrone.pos = old_pos;
    end

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

