from layered_planner_sim import obstacles, Robot, params, poses2polygons, get_info, t_array
import numpy as np
from copy import deepcopy
from numpy.linalg import norm

store_data = list()
t_array = [1, 2, 3]

attack_robot = Robot('attack')
attack_rob_obs = poses2polygons([attack_robot.sp])

robot1 = Robot(1)
robot1.sp = np.array([1.2, 1.0])
robot1.sp_global = np.array([0.8, 1.5])
robot1.local_planner(obstacles + attack_rob_obs, params)

# robot1.sp = np.array([1.2, 1.0])
# robot1.sp_global = np.array([0.8, 1.5])
# attack_robot.sp = np.array([ 0.03744805 -0.01405857])
# attack_rob_obs = poses2polygons([attack_robot.sp])
# robot1.local_planner(np.array(obstacles + attack_rob_obs), params)

# store_data.append(deepcopy(robot1.U))
# store_data.append(deepcopy(robot1.U_a))
# store_data.append(deepcopy(robot1.U_r))

print(robot1.monitor.multi_rule_monitor(get_info(robot1)))

# print(robot1.sp)
# print(robot1.V)

attack_robot.sp = np.array([1.2, 0.71])
# attack_robot.sp = np.array([0.9, 0.7])
attack_rob_obs = poses2polygons([attack_robot.sp])
robot1.sp = np.array([1.2, 1.0])
robot1.sp_global = np.array([0.8, 1.5])

print("距离是：")
print(norm(robot1.sp - attack_robot.sp))
robot1.local_planner(np.array(obstacles + attack_rob_obs), params)

print(robot1.monitor.multi_rule_monitor(get_info(robot1)))

# print(robot1.sp)
# print(robot1.V)

# for i in range(len(robot1.U)):
#     for j in range(len(robot1.U[i])):
#         if robot1.U[i][j] != store_data[0][i][j]:
#             print(i, j, robot1.U[i][j], store_data[0][i][j])
# for i in range(len(robot1.U_a)):
#     for j in range(len(robot1.U_a[i])):
#         if robot1.U_a[i][j] != store_data[1][i][j]:
#             print('hello')
# for i in range(len(robot1.U_r)):
#     for j in range(len(robot1.U_r[i])):
#         if robot1.U_r[i][j] != store_data[2][i][j]:
#             print('hello')