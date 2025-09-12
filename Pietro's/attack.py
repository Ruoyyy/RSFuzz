import numpy as np
import random
import robustness as rob
import math
import os
import katz
# os.sys.path().append('D:\\liyilin\\Swarm\\SwarmRoboticsSim\\')
##数据处理

def attack_agent_location(x, y):
    x_location, y_location = 0, 0
    for i in range(10):
        x_location += float(x[i])
        y_location += float(y[i])
    x_location = y_location/10
    y_location = y_location/10
    return [x_location, y_location]

def drones_agent_location(x, y):
    x, y= list(x), list(y)
    drones_agent_location = []
    for i in range(len(x)):
        drones_agent_location.append((float(x[i]), float(y[i])))
    return drones_agent_location

def obstacles_list(x, y):
    x, y= list(x), list(y)
    obstacles_list = []
    location_x, location_y = x[:-1], y[:-1]
    for i in range(len(location_x)):
        obstacles_list.append((float(location_x[i]), float(location_y[i])))
    return obstacles_list


# def attack_agent_location_random(attack_agent_location, attack_agent_v):
#     random_points= []
#     # 在攻击机的行驶范围内随机生成a个点
#     # 生成随机角度
#     for i in range(20):
#         angle = random.uniform(0, 2*math.pi)
#         # 计算随机点的坐标
#         x = attack_agent_location[0] + attack_agent_v * math.cos(angle)
#         y = attack_agent_location[1] + attack_agent_v * math.sin(angle)
#         random_points.append((x, y))
#     return random_points
    # pass
def attack_agent_location_random(drone_list, drone_agent_r, attack_agent_location, attack_agent_v, number, max_influence_distance):
    graph = katz.Graph(drone_list, max_influence_distance)
    drone_agent_location = drone_list[graph.vulnerable_nodes[0]]
    # print(drone_agent_location)  
    x1, y1 = drone_agent_location[0], drone_agent_location[1]
    x2, y2 = attack_agent_location[0], attack_agent_location[1]
    r1 = drone_agent_r
    r2 = attack_agent_v
    # 找出两个圆的交点
    def find_intersection_points(x1, y1, r1, x2, y2, r2):
        d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if d > r1 + r2 or d < abs(r1 - r2):
            return None  # 没有交点
        
        a = (r1**2 - r2**2 + d**2) / (2 * d)
        h = np.sqrt(r1**2 - a**2)
        x0 = x1 + a * (x2 - x1) / d
        y0 = y1 + a * (y2 - y1) / d
        rx = -(y2 - y1) * (h / d)
        ry = (x2 - x1) * (h / d)
        
        return [(x0 + rx, y0 + ry), (x0 - rx, y0 - ry)]
    # 找出交点
    intersection_points = find_intersection_points(x1, y1, r1, x2, y2, r2)
    # print(intersection_points)
    if intersection_points is None:
        print("攻击机与无人机没有重叠部分")
        return None
    else:
        # 在重叠区域内随机生成点
        random_points = []            
        for _ in range(int(number)):
            while True:
                x = np.random.uniform(min(x1 - r1, x2 - r2), max(x1 + r1, x2 + r2))
                y = np.random.uniform(min(y1 - r1, y2 - r2), max(y1 + r1, y2 + r2))
                if (x - x1)**2 + (y - y1)**2 <= r1**2 and (x - x2)**2 + (y - y2)**2 <= r2**2:
                    random_points.append((x, y))
                    break        
        return random_points

def find_influenced_drones(drone_agent_location, drone_agent_r, attack_agent_location, attack_agent_v):
    """
    找出与攻击机可达范围有重叠部分的无人机的无人机坐标
    """
    influenced_drones = []
    
    for center in drone_agent_location:
        dist = math.sqrt((center[0] - attack_agent_location[0]) ** 2 + (center[1] - attack_agent_location[1]) ** 2)
        if dist <= (drone_agent_r + attack_agent_v) and dist >= abs(drone_agent_r - attack_agent_v):
            influenced_drones.append(center)
    
    return influenced_drones

def attack_strategy_SA(attack_agent_location, drones_agent_location, obstacles_list):
    """
    攻击策略1：SA-Fuzzing
    """
    # print(len(attack_agent_location))
    robustness = [0 for i in range(len(attack_agent_location))]
    obstacles_list = list(obstacles_list)
    i = 0
    for attack in attack_agent_location:
        x, y = attack[0], attack[1]
        # print(x, y)
        obstacles_list.append([(x-1, y),(x, y-1),(x+1, y),(x, y+1)])
        # 计算每个随机点的robustness值
        robustness[i] = rob.rule1_prepare(drones_agent_location, obstacles_list)
        #obstacles_list.remove([(x-1, y),(x, y-1),(x+1, y),(x, y+1)])
        obstacles_list.pop()
        i += 1
    # print(robustness)
    attack_index = robustness.index(min(robustness))
    return attack_agent_location[attack_index]


def attack_strategy_MA(drones_agent_location, drones_agent_r, obstacles_list):
    """
    攻击策略2：MA-Fuzzing
    """
    graph = katz.Graph(drones_agent_location, 1.5*drones_agent_r)
    vulnerable_nodes = drones_agent_location[graph.vulnerable_nodes[0]]
    # print(vulnerable_nodes)
    attack_agent_locations = []
    for i in range(20):
        angle = random.uniform(0, 2 * math.pi)
        x = vulnerable_nodes[0] + drones_agent_r * math.cos(angle)
        y = vulnerable_nodes[1] + drones_agent_r * math.sin(angle)
        attack_agent_locations.append((x, y))
    min_robustness = [0 for i in range(len(drones_agent_locations))]
    attack = [0 for i in range(len(drones_agent_locations))]
    robustness = [0 for i in range(20)]
    obstacles_list = list(obstacles_list)
    # 随机在无人机的周围生成20个点 
    m = 0
    for attack_points in attack_agent_locations:
        i = 0 
        for attack_point in attack_points:
            x, y = attack_point[0], attack_point[1]
            obstacles_list.append([(x-1, y),(x, y-1),(x+1, y),(x, y+1)])
            # 计算每个随机点的robustness值
            robustness[i] = rob.rule1_prepare(drones_agent_location, obstacles_list)
            obstacles_list.remove([(x-1, y),(x, y-1),(x+1, y),(x, y+1)])
            i += 1
        attack[m] = attack_points[robustness.index(min(robustness))]
        min_robustness[m] = min(robustness)
        m += 1
    attack_agent_location = attack[min_robustness.index(min(min_robustness))]
    return attack_agent_location
            
            






