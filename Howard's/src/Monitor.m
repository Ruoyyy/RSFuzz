function success = Monitor(drones, obstacles, end_loc, max_time)
    % 初始化成功标志
    success = false;
    
    % 检查是否任意一架无人机撞到障碍物
    for i = 1:length(drones)
        for j = 1:length(obstacles)
            distance = norm(drones(i).pos - obstacles(j).pos);
            if distance < 1
                success = true;
                return;
            end
        end
    end
    
    % 检查无人机之间是否相互碰撞
    for i = 1:length(drones)
        for j = i+1:length(drones)
            distance = norm(drones(i).pos - drones(j).pos);
            if distance < 1
                success = true;
                return;
            end
        end
    end
    
    % 检查是否长时间未到达目的地
    for i = 1:length(drones)
        if norm(drones(i).pos - end_loc) > 0.1 && max_time <= 0
            success = true;
            return;
        end
    end
end
