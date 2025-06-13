% 打开文件
fileID = fopen('ablation_points.txt', 'r');
% 循环读取每行数据
tline = fgetl(fileID);
k=0;
while ischar(tline)
    k =k+1; 
    % 根据逗号分割字符串
    coords_str = strsplit(tline, ',');
    x = str2double(coords_str{1});
    y = str2double(coords_str{2});
    ablation{k} = [x, y];
    % 显示坐标
    % display (ablation{k})
    % 读取下一行
    tline = fgetl(fileID);
end

% 关闭文件
fclose(fileID);
for z = 1:200
    %% Configuration file for quadrotor_swarm.m
    %Enter simulation parameter values below
    N = 10;         %number of agents
    t0 = 0;         %initial time (s)
    t_end = 2000;     %end time (s)
    timer = 1;    %agent timer s(1) / S(1,j). Can be used to transition from initial state
    dt = 5e-1;      %simulation time step (s)
    
    %Try to keep values below unchanged
    r_agent = 2;    %nearby agent detection/communication radius (m)
    IR_angle = pi*2; %infrared proximity sensor angle
    IR_dist = 2;    %infrared proximity sensor range (m)
    target_radius = 1;    %radius of target(s)
    v_max = 2;      %max quadcopter velocity (m/s)
    d_max = v_max*dt; %maximum agent travel displacement per time step
    % Check time step is sufficiently small:
    if d_max >= IR_dist
        disp('Warning: time step too large, may cause agents to crash into obstacles')    
    end
    R_search = 0.02; %target search rate (/s)
    %%Attack
    % attack_agent_location = [10 20];
    attack_agent_location = ablation{z};
    attack_agent_v = 1;
    
    %% Terrain/obstacle properties
    
    map_type = 0;   % = 0 for standard map, = 1 for custom map
    
    if map_type == 0;
            load hard_obstacles.mat;
        load hard_targets.mat;
        %load obstacles_standard.mat;  %rename as map_standard.mat or similar
        %load targets_standard.mat;
        map_size = 50;
    else
        [map_size] = map_maker(1); 
        load hard_obstacles.mat;
        load hard_targets.mat;
    end
    try
        SA_Fuzzing
    catch
        continue
    end
end


