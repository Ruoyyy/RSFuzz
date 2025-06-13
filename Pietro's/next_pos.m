function [ Q  ] = next_pos(t,map_size,IR_dist,IR_angle,r_agent,targets,target_status,target_radius,obstacles,N,d_max,Q,S,neighbour)

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