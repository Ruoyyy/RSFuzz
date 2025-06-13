import numpy as np
def SwarmDCC(pos_old, pos_new, Dornes_len, Drone_index, Obstacles_len):
    pos_old = np.array(pos_old)
    pos_new = np.array(pos_new)
    # print(pos_old)
    # print(pos_new)
    Dornes_len = int(Dornes_len)
    Obstacles_len = int(Obstacles_len)
    Drone_index = int(Drone_index)
    # print(Dornes_len)
    # print(Obstacles_len)
    detal = [0]*(Dornes_len+Obstacles_len)
    # print(detal)
    Total = 0
    DCC = [0]*(Dornes_len+Obstacles_len)
    for j in range(Obstacles_len):
        # print(pos_new[j])
        detal[j] = np.linalg.norm(pos_new[j]-pos_old)
        # print(detal[j])
        Total += detal[j]
        # print(Total)
    for k in range(Dornes_len):
        pos_new[Obstacles_len+k] = np.array(pos_new[Obstacles_len+k])
        # print(pos_new[Obstacles_len+k])
        detal[Obstacles_len+k] = np.linalg.norm(pos_new[Obstacles_len+k]-pos_old)
        Total += detal[Obstacles_len+k]
    for m in range(Obstacles_len+Dornes_len):
        # print(detal[m])     
        DCC[m] = detal[m]/Total
    del DCC[Obstacles_len+Drone_index-1]
    return DCC
def attack(attack_agent, drones_pos, strategy):
    if strategy == 1:
        # 攻击机的目标位置是一号无人机x轴前0.2m
        target_pos = (drones_pos[0][0] + 0.2, drones_pos[0][1])
    elif strategy == 2:
        # 攻击机的目标位置是一号无人机x轴后0.1m
        target_pos = (drones_pos[0][0] - 0.1, drones_pos[0][1])
    elif strategy == 3:
        # 攻击机的目标位置是一号无人机和三号无人机的中心点
        center_x = (drones_pos[0][0] + drones_pos[2][0]) / 2
        center_y = (drones_pos[0][1] + drones_pos[2][1]) / 2
        target_pos = (center_x, center_y)
    elif strategy == 4:
        # 对 x 坐标最小的无人机位置的 x 坐标减去 0.2
        # 对 y 坐标最大的无人机位置的 y 坐标加上 0.2
        min_x_drone = min(drones_pos, key=lambda pos: pos[0])
        max_y_drone = max(drones_pos, key=lambda pos: pos[1])
        target_pos = (min_x_drone[0] - 0.2, max_y_drone[1] + 0.2)
    else:
        raise ValueError("Invalid strategy")
    
    return target_pos
def move_to_target(attack_agent_pos, target_pos, attack_agent_v, dt):
    # 计算当前位置和目标位置的差值
    dx = target_pos[0] - attack_agent_pos[0]
    dy = target_pos[1] - attack_agent_pos[1]
    
    # 计算移动方向的单位向量
    distance = (dx**2 + dy**2)**0.5
    if distance == 0:
        return attack_agent_pos  # 已经在目标位置
    
    direction_x = dx / distance
    direction_y = dy / distance
    
    # 计算在dt时间步长内的移动距离
    move_distance = attack_agent_v * dt
    
    # 更新攻击代理的位置
    new_position = (attack_agent_pos[0] + direction_x * move_distance,
                    attack_agent_pos[1] + direction_y * move_distance)
    
    return new_position
def GetNCC(comparison_1, comparison_2):
    end_tick = len(comparison_1)
    num_objects = len(comparison_1[0])
    sum_temp_1 = 0
    sum_temp_2 = 0
    sum_temp_3 = 0
    distance = 0
    for object_index in range(num_objects):
        for tick_index in range(end_tick):
            temp_temp_1 = comparison_1[tick_index][object_index] * comparison_2[tick_index][object_index]
            sum_temp_1 += temp_temp_1

            temp_temp_2 = math.pow(comparison_1[tick_index][object_index], 2)
            sum_temp_2 += temp_temp_2

            temp_temp_3 = math.pow(comparison_2[tick_index][object_index], 2)
            sum_temp_3 += temp_temp_3
        
        distance = sum_temp_1 / math.sqrt(sum_temp_2 * sum_temp_3)
    return distance

            
