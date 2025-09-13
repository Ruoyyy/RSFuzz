import numpy as np
from shapely.geometry import Polygon, Point


def rule1_prepare(drones_agent_location, obstacles_location): 
    '''
    规则1，无人机不能撞向障碍物
    '''
    robuteness = [0 for i in range(len(drones_agent_location))]
    for i in range(len(drones_agent_location)):
        robuteness[i] = drone_to_obstacles_distance(drones_agent_location[i],obstacles_location)

    total_rob = sum(robuteness)
    return total_rob


def drone_to_obstacles_distance(drones_agent_location, obstacles_list):
      
    # 定义无人机以及最小距离
    drone = Point(drones_agent_location)
    min_distace = float('inf')
    # total_distance = 0
    for obstacles_points in obstacles_list:
         # 定义障碍物
        obstacles = Polygon(obstacles_points)
        # 计算无人机到障碍物的距离
        distance = drone.distance(obstacles)
        min_distace = min(min_distace, distance)
        # total_distance += distance
    return min_distace
    # return total_distance
    
# 判断攻击机是否与受害者无人机碰撞
def drone_is_crush(drone_point, obstacles_list):
    for drone in drone_point:
        if drone_to_obstacles_distance(drone, obstacles_list) == 0:
            return True
    return False

