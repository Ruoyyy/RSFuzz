%% Simulation parameters
% run simulation_config  %Change simulation parameters in simulation_config.m
% 设置 Python 解释器路径

pyenv('Version','D:\\Frame_envir\\anaconda3\\envs\\zry-python3.10\\python.exe');
python_module_path = 'D:\liyilin\Swarm\SwarmRoboticsSim\';
%py.importlib.import_module('attack.py');
% 添加 Python 模块路径
if isfolder(python_module_path)
    py.sys.path().append(python_module_path);
else
    disp('Error: Python module path does not exist.');
end
%% Initialise swarm state matrix - q(:,j) = [x y]
global Q
global S
Q = zeros(4,N); %create empty swarm state array
S = zeros(11,N); %create empty swarm sensor array
S(1,:) = timer; %input timer value into sensor array
countMatrix = zeros(1,N); %collisions between drones

%% Base location coordinates and target status
for i = 1:N;
    Q(2,i) = map_size/2; %initial x position (m)
    %Q(2,i) = 1.2 ;
    %Q(3,i) = 1.25;
    Q(3,i) = map_size/2; %initial y position (m)
end
target_status = zeros(size(targets,2),3); %create array to store search status (:,1), completion time of each target (:,2) and number of agents inside (:,3)
target_status(:,2) = NaN;   %initialise target 100% completion times as NaN

%% Calculations and plotting
t = t0;
figure(1);
whitebg([1 1 1]);
axis equal;
axis([0 map_size 0 map_size]);
xlabel('x (m)');
ylabel('y (m)');
ho = text(map_size/20,map_size/20,strcat(num2str(t),'s'));

for i = 1:((t_end/dt)+1);
figure(1);
target_status(:,3) = 0; %reset searching agents in target_status
if i >=3 
    %%AttackDrone
    %攻击机的坐标
    drones_agent_center = py.attack.drones_agent_location(Q(2,:), Q(3,:));
    for k = 1:size(obstacles,2)/2   
        obstacles_list{k} = py.attack.obstacles_list(obstacles{1,(2*k - 1)}(:),obstacles{1,(2*k)}(:));
    end
    
    % attack_agent_random = py.attack.attack_agent_location_random(attack_agent_location, attack_agent_v);
    % for i =1:N
    %     attack_agent_random{i} = py.attack.attack_agent_location_random(drones_agent_center{i}, IR_dist);
    % end
    
    m = 1;
    % while m < length(attack_agent_random)
    %     Attack_agnet_x(1:5) = {[attack_agent_random{m}{1}-1,attack_agent_random{m}{1},attack_agent_random{m}{1}+1,attack_agent_random{m}{1},attack_agent_random{m}{1}-1]};
    %     Attack_agnet_x(6:24) = {[]};
    %     Attack_agnet_y(1:5) = {[attack_agent_random{m}{2},attack_agent_random{m}{2}-1,attack_agent_random{m}{2},attack_agent_random{m}{2}+1,attack_agent_random{m}{2}]};
    %     Attack_agnet_y(6:24) = {[]};
    %     Attack_agnet = [Attack_agnet_x',Attack_agnet_y'];
    %     obstacles(:,9:10) = Attack_agnet;
    %     [New_Q] = next_pos(t ,map_size ,IR_dist ,IR_angle ,r_agent ,targets , target_status, target_radius, obstacles, N,d_max, Q, S, neighbour);
    %     crash = inpolygon(New_Q(2,:),New_Q(3,:),obstacles{1,(9)}(:),obstacles{1,(10)}(:));
    %     %if py.robustness.drone_is_crush(py.attack.drones_agent_location(New_Q(2,:),New_Q(3,:)), {py.attack.obstacles_list((obstacles{1,(9)}(:)),obstacles{1,(10)}(:))}) == 1
    %     %if inpolygon(New_Q(2,:),New_Q(3,:),obstacles{1,(9)}(:),obstacles{1,(10)}(:)) == 1
    %     if isempty(find(crash)) == 0
    %         attack_agent_random.remove(attack_agent_random{m})
    %     else
    %         m = m+1;
    %     end
    % end
    % obstacles(:,9:10) = [];
    attack_agent_next = py.attack.attack_strategy_2(attack_agent_random, drones_agent_center, obstacles_list);
    Attack_agnet_x(1:5) = {[attack_agent_next{1}-1,attack_agent_next{1},attack_agent_next{1}+1,attack_agent_next{1},attack_agent_next{1}-1]};
    Attack_agnet_x(6:24) = {[]};
    Attack_agnet_y(1:5) = {[attack_agent_next{2},attack_agent_next{2}-1,attack_agent_next{2},attack_agent_next{2}+1,attack_agent_next{2}]};
    Attack_agnet_y(6:24) = {[]};
    Attack_agnet = [Attack_agnet_x',Attack_agnet_y'];
    obstacles(:,9:10) = Attack_agnet;
    % [New_Q] = next_pos(t ,map_size ,IR_dist ,IR_angle ,r_agent ,targets , target_status, target_radius, obstacles, N,d_max, Q, S, neighbour);
    % t 
    % map_size
    % IR_dist 
    % IR_angle 
    % r_agent 
    % targets 
    % target_status
    % target_radius
    % obstacles
    % 4
    % d_max
    % Q
    % S 
    % neighbour
    %% next_crash = inpolygon(New_Q(2,:),New_Q(3,:),obstacles{1,(9)}(:),obstacles{1,(10)}(:))
    attack_agent_location = attack_agent_next;
end


%% Calculate sensor values for current position of agent j
for j = 1:N    %cycle through swarm of agents
    
    %% Define IR sensors
    %Left/right IR proxity sensors 
    IR_right = [Q(2,j) (Q(2,j)+IR_dist*cos(Q(4,j))) (Q(2,j)+IR_dist*cos(Q(4,j)-IR_angle)) Q(2,j); Q(3,j) (Q(3,j)+IR_dist*sin(Q(4,j))) (Q(3,j)+IR_dist*sin(Q(4,j)-IR_angle)) Q(3,j)]; 
    IR_left = [Q(2,j) (Q(2,j)+IR_dist*cos(Q(4,j))) (Q(2,j)+IR_dist*cos(Q(4,j)+IR_angle)) Q(2,j); Q(3,j) (Q(3,j)+IR_dist*sin(Q(4,j))) (Q(3,j)+IR_dist*sin(Q(4,j)+IR_angle)) Q(3,j)];
    S(2,j) = 0; %reset right IR sensor value to 0
    S(3,j) = 0; %reset left IR sensor value to 0
%   plot(IR_left(1,:),IR_left(2,:),'-g');    %plot left IR sensor
%   hold on;
%   plot(IR_right(1,:),IR_right(2,:),'-r');  %plot right IR sensor
    
    %% Obstacle detection
    for k = 1:size(obstacles,2)/2;        
        [IR_detect] = polyxpoly(IR_right(1,:),IR_right(2,:),obstacles{1,(2*k - 1)}(:),obstacles{1,(2*k)}(:));
        if  IR_detect
             S(2,j) = 1; %obstacle detected right, set right IR sensor to 1
        end
        [IR_detect] = polyxpoly(IR_left(1,:),IR_left(2,:),obstacles{1,(2*k - 1)}(:),obstacles{1,(2*k)}(:));
        if  IR_detect
             S(3,j) = 1; %obstacle detected left, set left IR sensor to 1
        end
    end
    
    %% Map boundary detection
    [IR_detect] = polyxpoly(IR_right(1,:),IR_right(2,:),[1E-3; 1E-3; map_size; map_size; 1E-3],[1E-3; map_size; map_size; 1E-3; 1E-3]);
    %[IR_detect] = polyxploy(IR_right(1,:),IR_right(2,:),[-2.5; -2.5; 2.5; 2.5; -2.5],[-2.5; 2.5; 2.5; -2.5; -2.5]);
        if  IR_detect
             S(2,j) = 1; %map boundary detected right, set right IR sensor to 1
        end
    [IR_detect] = polyxpoly(IR_left(1,:),IR_left(2,:),[1E-3; 1E-3; map_size; map_size; 1E-3],[1E-3; map_size; map_size; 1E-3; 1E-3]);
    %[IR_detect] = polyxploy(IR_letf(1,:),IR_letf(2,:),[-2.5; -2.5; 2.5; 2.5; -2.5],[-2.5; 2.5; 2.5; -2.5; -2.5]);
        if  IR_detect
             S(3,j) = 1; %map boundary detected left, set left IR sensor to 1          
        end
    
    %% Detect nearby agent
    % dseachn function is used to find nearest agent
    % neighbour is the index of the nearest agent
    % d_agent is the distance to the nearest agent
    if j == 1;
        [neighbour,d_agent] = dsearchn(Q(2:3,2:N)',Q(2:3,j)');
    elseif j == N;
        [neighbour,d_agent] = dsearchn(Q(2:3,(1:(j-1)))',Q(2:3,j)');
    else
        [neighbour,d_agent] = dsearchn(Q(2:3,[(1:(j-1)) ((j+1):N)])',Q(2:3,j)');
    end
        
    if neighbour >= j;
        neighbour = neighbour +1;  %ensure neighbour index is correct 
    end
    
    if d_agent <= r_agent; %neighbouring agent is not within communication range
        S(9,j) = 1;         %agent detection set to 1 in sensor array
        S(10,j) = d_agent;  %agent-agent distance input to sensor array
        S(11,j) = acos((Q(2,neighbour)-Q(2,j))/d_agent);   %agent-agent angle input to sensor array
        if (Q(3,neighbour)-Q(3,j)) < 0;   
            S(11,j) = S(11,j)*-1; %ensure angle sign is correct
        end
%        plot([Q(2,j) Q(2,neighbour)],[Q(3,j) Q(3,neighbour)],'-m'); %plot solid line between nearby agents within comms range
%        hold on;
    else    %neighbouring agent is not within communication range
        S(9,j) = 0; %agent detection set to 0 in sensor array
        S(10,j) = 0; %non-existent agent-agent distance input to sensor array
        S(11,j) = 0; %non-existent agent-agent angle input to sensor array
%        plot([Q(2,j) Q(2,neighbour)],[Q(3,j) Q(3,neighbour)],':m'); %plot dotted line between nearby agents outside comms range
%        hold on;
        neighbour = NaN;  %nearby agent is out of communcation range so cannot be accessed in agent_control.m
    end

    %% Proximity to target
    S(6,j) = 0; %reset target sensor value to 0
    [nearby_target,d_target] = dsearchn(targets',Q(2:3,j)');    
    
    if target_status(nearby_target,1) < 1;
        if d_target <= target_radius;   %target is detectable
            S(6,j) = 1; %set target sensor value to 1
            S(7,j) = d_target;
            if Q(2,j) >= targets(1,nearby_target) && Q(3,j) >= targets(2,nearby_target);
                 S(8,j) = acos(abs(targets(1,nearby_target)-Q(2,j))/d_target)+pi;
            elseif Q(2,j) <= targets(1,nearby_target) && Q(3,j) >= targets(2,nearby_target);
                 S(8,j) = (2*pi)-acos(abs(targets(1,nearby_target)-Q(2,j))/d_target);
            elseif Q(2,j) <= targets(1,nearby_target) && Q(3,j) <= targets(2,nearby_target)
                 S(8,j) = acos(abs(targets(1,nearby_target)-Q(2,j))/d_target);
            else
                 S(8,j) = (pi - acos(abs(targets(1,nearby_target)-Q(2,j))/d_target));
            end
        end
    end        

    %% Input state and sensor data into agent logic 
    pos_old = Q(2:3,j); %temporarily store old position
    %这里需要还原无人机现场
    [Q(:,j)] = advanced_agent_control(t,j,N,d_max,Q(:,j),S(:,j),neighbour);    %run agent_control sub-function for agent j
    S(4,j) = sqrt((pos_old(1) - Q(2,j))^2 + (pos_old(2) - Q(3,j))^2);  %enter agent translation distance into sensor array
    S(5,j) = Q(4,j); %enter agent theta into sensor array
end

% if i > 10
%     if ~isequal(Q, New_Q)
%         i
%         Q
%         New_Q
%         t 
%         map_size
%         IR_dist 
%         IR_angle 
%         r_agent 
%         targets 
%         target_status
%         target_radius
%         obstacles 
%         4
%         d_max
%         Q
%         S 
%         neighbour
%     end
% end
%% Plotting
 plot(Q(2,:),Q(3,:),'db'); %plot quadrotor agents
 hold on;
 plot(Q(2,:),Q(3,:),'xb'); %plot quadrotor agents
 hold on;
 for j = 1:N
     rectangle('Position', [Q(2,j)-IR_dist,Q(3,j)-IR_dist,2*IR_dist,2*IR_dist], 'Curvature', [1 1],'EdgeColor', 'r');
     %viscircles([Q(2,j) Q(3,j)],IR_dist,'Color','r');
     axis equal
 end
 %plot(1.2,1.25,'sk','MarkerFaceColor','c'); %plot base location
 %% Crash detection
 in_obstacles = cell(size(obstacles,2)/2,1);
 for k = 1:size(obstacles,2)/2-1;
     in_obstacles{k} = inpolygon(Q(2,:),Q(3,:),obstacles{1,(2*k - 1)}(:),obstacles{1,(2*k)}(:));
     for b = find(in_obstacles{k}); %find agents in obstacles
         Q(1,b) = 3; %State = 3 (crashed, terminal state)
         display(t)
         filename_txt = 'MA_10_result.txt'; % 文件名
         fid = fopen(filename_txt, 'a'); % 打开文件以供写入
         fprintf(fid, '%f\n', t); % 将变量的值写入文件
         fclose(fid); % 关闭文件
         error("集群无人机撞向障碍物")
         plot(Q(2,b),Q(3,b),'dk');
         plot(Q(2,b),Q(3,b),'xk');
     end
 end
 
 in_map = inpolygon(Q(2,:),Q(3,:),[1E-3 map_size map_size 1E-3],[1E-3 1E-3 map_size map_size]);  %find agents outside map boundary
 %in_map = inpolygon(Q(2,:),Q(3,:),[-2.5 2.5 2.5 -2.5],[-2.5 -2.5 2.5 2.5]);
  if ~in_map(:) == zeros(N,1)
      %all agents within map boundaries => do nothing
  else
      %find agents outside map boundaries => crash them
      for b = find(~in_map);
         Q(1,b) = 3; %State = 3 (crashed, terminal state)
         display(t)
         filename_txt = 'MA_10_result.txt'; % 文件名
         fid = fopen(filename_txt, 'a'); % 打开文件以供写入
         fprintf(fid, '%d\n', t); % 将变量的值写入文件
         fclose(fid); % 关闭文件
         error("集群无人机撞向边界")
         plot(Q(2,b),Q(3,b),'dk');
         plot(Q(2,b),Q(3,b),'xk');
         axis equal
      end
  end

 %%collisions between drones
if i >= 10
     for i = 1:N
         for j = (i+1):N
             if abs(Q(2,i) - Q(2,j))<=0.5&&abs(Q(3,i) - Q(3,j))<=0.5
                 countMatrix(i) = countMatrix(i) + 1;
             end
         end
     end

    % disp(countMatrix);
end


 %% More plotting
ho = text(map_size/20,map_size/20,strcat(num2str(t),'s'));
axis equal;
%axis([-2.6 2.6 -2.6 2.6]);
axis([0 map_size 0 map_size]);
xlabel('x (m)');
ylabel('y (m)');
for k = 1:size(obstacles,2)/2;
    facecolour = zeros(1,size(obstacles{1,(2*k - 1)}(:),1));
    patch((obstacles{1,(2*k - 1)}(:))',(obstacles{1,(2*k)}(:))',facecolour,'EdgeAlpha',0.2,'FaceColor','g','FaceAlpha',0.2);
    whitebg([1 1 1]);
end

%% Target search calculation and display
agents_inside = find(Q(1,:)==2); %find agents in State 2 - Search

for k = agents_inside; %cycle through each agent that has entered a target
    [entered_target,d_target] = dsearchn(targets',Q(2:3,k)');  %find which target is being searched
    if d_target <= target_radius; %double check agent is actually above target and not just in wrong FSA state
        target_status(entered_target,3) = target_status(entered_target,3) + 1;
    end
end

for k = 1:size(targets,2);
    target_status(k,1) = target_status(k,1) + (target_status(k,3)*R_search*dt);
    if target_status(k,1) < 1;
        rectangle('Position',[targets(1,k)-target_radius targets(2,k)-target_radius 2*target_radius 2*target_radius],'Curvature',[1,1],'FaceColor','r');
    else
        target_status(k,1) = 1;
        if isnan(target_status(k,2)) == 1;
            target_status(k,2) = t; %store time of target search completion
        end
        rectangle('Position',[targets(1,k)-target_radius targets(2,k)-target_radius 2*target_radius 2*target_radius],'Curvature',[1,1],'FaceColor','w');
    end
    text((targets(1,k)+(map_size/100)),(targets(2,k)+(map_size/100)),strcat(num2str(100*target_status(k,1)),'%'));
end

for k = agents_inside; %cycle through each agent that has entered a target
    [entered_target] = dsearchn(targets',Q(2:3,k)');  %find which target is being searched
    if target_status(entered_target,1) >= 1;    %check if target has 100% completion
        S(6,k) = 0; %target is 100% searched to reset target detection to 0 (allows transition to other state)
    end
end

%% Save screengrab of Figure 1 as image

if sum(target_status(:,1)) == size(target_status,1);
name = strcat('Advanced_agent_control_completed_swarm_map',num2str(t),'s.bmp'); %create filename for screengrab
p = figure(1);
print(p,'-dbmp',name);  %save Figure(1) at time t as a bitmap image file

break

end    
% name = strcat('swarm_movie',num2str(t),'s.bmp'); %create filename for screengrab
% p = figure(1);
% print(p,'-dbmp',name);  %save Figure(1) at time t as a bitmap image file
%name = strcat('AAAAmedium_completed_swarm_map',num2str(t),'s.bmp'); %create filename for screengrab
%p = figure(1);
%print(p,'-dbmp',name);  %save Figure(1) at time t as a bitmap image file


%% Prepare for next time step
hold off;
t = t + dt; %next time step
target_status;
end
