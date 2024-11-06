#!/usr/bin/env python

"""
Autonumous navigation of robots formation with Layered path-planner:
- global planner: RRT
- local planner: Artificial Potential Fields
"""
import sys
sys.path.append('/home/czh/attack/')
from tool.tool import Monitor
from svgpath2mpl import parse_path

import matplotlib
import numpy as np
from numpy.linalg import norm

from tools import *
from rrt import *
from potential_fields import *
import time

# for 3D plots
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
matplotlib.use('Qt5Agg')

import random
import math
from copy import deepcopy

from data_collection.path import P_path, traj_global_path

t_array = [0]

nrows, ncols = 500, 500
grid = np.zeros((nrows, ncols))

is_crash = False

def move_obstacles(obstacles, params):
    # small cubes movement
    obstacles[-3] += np.array([0.015, 0.0]) * params.drone_vel
    obstacles[-2] += np.array([-0.005, 0.005]) * params.drone_vel/2
    obstacles[-1] += np.array([0.0, 0.008]) * params.drone_vel/2

    # if params.dir == -1:
    #     for i in range(25):
    #         obstacles[i] += np.array([0.0, -0.008])
    #     # obstacles[11] += np.array([0.0, -0.008])
    #     # obstacles[18] += np.array([0.0, -0.008])
    #     params.dir = 1
    # else:
    #     for i in range(25):
    #         obstacles[i] += np.array([0.0, 0.008])
    #     # obstacles[11] += np.array([0.0, 0.008])
    #     # obstacles[18] += np.array([0.0, 0.008])
    #     params.dir = -1


    # if params.dir == -1:
    #     for i in range(25):
    #         obstacles[i] += np.array([0.008, -0.008])
    #     # params.dir = 1
    # else:
    #     for i in range(25):
    #         obstacles[i] += np.array([0.0, 0.008])
    #     params.dir = -1
    return obstacles

class Params:
    def __init__(self):
        self.animate_rrt = 1 # show RRT construction, set 0 to reduce time of the RRT algorithm
        self.visualize = 1 # show robots movement
        self.postprocessing = 0 # process and visualize the simulated experiment data after the simulation
        self.savedata = 0 # save postprocessing metrics to the XLS-file
        self.maxiters = 500 # max number of samples to build the RRT
        self.goal_prob = 0.05 # with probability goal_prob, sample the goal
        self.minDistGoal = 0.25 # [m], min distance os samples from goal to add goal node to the RRT
        self.extension = 0.8 # [m], extension parameter: this controls how far the RRT extends in each step.
        self.world_bounds_x = [-2.5, 2.5] # [m], map size in X-direction
        self.world_bounds_y = [-2.5, 2.5] # [m], map size in Y-direction
        self.drone_vel = 4.0 # [m/s]
        self.ViconRate = 100 # [Hz]
        self.influence_radius = 0.15 # [m] potential fields radius, defining repulsive area size near the obstacle
        self.goal_tolerance = 0.05 # [m], maximum distance threshold to reach the goal
        self.num_robots = 4 # number of robots in the formation
        self.interrobots_dist = 0.3 # [m], distance between robots in default formation
        self.max_sp_dist = 0.2 * self.drone_vel# * np.sqrt(self.num_robots) # [m], maximum distance between current robot's pose and the sp from global planner
        self.dir = 0

class Robot:
    def __init__(self, id, monitor=None):
        self.id = id
        self.sp = np.array([0, 0])
        self.sp_global = np.array([0,0])
        self.route = np.array([self.sp])
        self.vel_array = []
        self.U_a = 0 # attractive APF function
        self.U_r = 0 # repulsive APF function
        self.U = 0 # total APF function
        self.leader = False
        self.perceptual_robustness_list = []
        self.curve_count = 5
        self.straight_count = 5
        self.no_influence_rob = 0.0
        if not monitor:
            self.monitor = Monitor('uav' + str(self.id))
        else:
            self.monitor = monitor
        # self.cal_no_influence_rob()

    def local_planner(self, obstacles, params):
        """
        This function computes the next_point
        given current location (self.sp) and potential filed function, f.
        It also computes mean velocity, V, of the gradient map in current point.
        """
        obstacles_grid = grid_map(obstacles)
        self.U, self.U_a, self.U_r = combined_potential(obstacles_grid, self.sp_global, params.influence_radius)
        [gy, gx] = np.gradient(-self.U)
        iy, ix = np.array( meters2grid(self.sp), dtype=int )
        w = 20 # smoothing window size for gradient-velocity
        ax = np.mean(gx[ix-int(w/2) : ix+int(w/2), iy-int(w/2) : iy+int(w/2)])
        ay = np.mean(gy[ix-int(w/2) : ix+int(w/2), iy-int(w/2) : iy+int(w/2)])
        # ax = gx[ix, iy]; ay = gy[ix, iy]
        self.V = params.drone_vel * np.array([ax, ay])
        self.vel_array.append(norm(self.V))
        dt = 0.01 * params.drone_vel / norm([ax, ay]) if norm([ax, ay])!=0 else 0.01
        # self.sp += dt**2/2. * np.array( [ax, ay] )
        self.sp = self.sp + dt*np.array( [ax, ay] ) #+ 0.1*dt**2/2. * np.array( [ax, ay] )
        self.route = np.vstack( [self.route, self.sp] )

    def cal_no_influence_rob(self):
        # 计算无人机不受影响的位置的健壮性，默认位置选择四个角落
        corner_list = [[2.5, 2.5], [2.5, -2.5], [-2.5, 2.5], [-2.5, -2.5]]

        # 保存位置
        rbt_sp = self.sp
        rbt_route = self.route

        for corner in corner_list:
            # 计算角落到无人机的距离
            dis = norm(corner - self.sp)
            if dis > params.influence_radius:
                # 距离超过无人机感知范围
                point_obs = poses2polygons(corner)
                self.local_planner(np.array(obstacles + point_obs), params)
                self.no_influence_rob = self.monitor.multi_rule_monitor(get_info(self))
                self.monitor.logger.debug("无人机感知范围外的健壮性为%s" % self.no_influence_rob)
                self.sp = rbt_sp
                self.sp_global = rbt_route
                return 
            else:
                continue

def get_info(robot: Robot) -> dict:
    info = dict()
    info['uav_pos'] = robot.sp
    info['obs_list'] = obstacles[:-4]
    info['DIS_PRE_MIN_OBS'] = params.influence_radius
    info['time'] = t_array[-1]
    return info

def visualize2D():

    global is_crash

    smiley = parse_path("""M458 2420 c-215 -38 -368 -257 -329 -469 34 -182 175 -314 354 -329 l57 -4 0 45 0 44 -42 7 c-101 16 -187 79 -236 171 -37 69 -38 187 -4 257 30 60 90 120 150 150 70 34 188 33 258 -4 89 -47 153 -136 169 -235 l7 -43 50 0 51 0 -6 59 c-13 147 -124 285 -268 334 -60 20 -152 28 -211 17z M1940 2417 c-172 -39 -302 -181 -317 -347 l-6 -60 51 0 50 0 12 52 c14 70 49 126 110 181 118 106 284 100 399 -14 64 -64 86 -120 86 -214 0 -67 -5 -88 -27 -130 -49 -92 -135 -155 -236 -171 l-42 -7 0 -49 0 -50 58 4 c115 8 242 91 306 200 36 61 59 177 51 248 -30 244 -260 410 -495 357z M506 2038 c-9 -12 -16 -41 -16 -64 0 -39 11 -56 158 -240 87 -110 161 -205 166 -212 5 -9 10 -382 6 -494 0 -3 -74 -97 -165 -208 l-165 -202 0 -52 c0 -68 18 -86 86 -86 40 0 55 5 80 28 17 15 112 89 211 166 l180 138 239 0 239 -1 209 -165 c203 -162 210 -166 256 -166 60 0 80 20 80 81 0 43 -8 55 -170 264 l-170 220 0 230 c0 202 2 233 18 257 9 15 86 108 170 208 l152 180 0 54 c0 65 -19 86 -76 86 -36 0 -58 -15 -234 -151 -107 -83 -205 -158 -217 -166 -19 -12 -67 -15 -260 -15 l-238 1 -209 165 -209 166 -53 0 c-43 0 -56 -4 -68 -22z M415 926 c-199 -63 -321 -258 -286 -457 31 -179 161 -309 340 -340 75 -14 171 1 248 37 116 55 209 188 220 314 l6 60 -49 0 -49 0 -17 -70 c-20 -84 -62 -147 -123 -188 -154 -102 -363 -44 -446 124 -35 72 -34 189 3 259 49 92 135 155 236 171 l42 7 0 48 0 49 -42 -1 c-24 0 -61 -6 -83 -13z M2020 882 l0 -50 43 -7 c99 -16 188 -80 235 -169 22 -43 27 -64 27 -131 0 -98 -23 -155 -90 -219 -177 -172 -471 -67 -511 183 l-7 41 -50 0 -50 0 6 -60 c11 -126 102 -257 218 -314 251 -123 542 26 590 303 39 221 -132 448 -351 468 l-60 6 0 -51z""")
    smiley.vertices -= smiley.vertices.mean(axis=0)

    draw_map(obstacles)
    theta = np.linspace(0, 2 * np.pi, 1000)
    # draw_gradient(robots[1].U) if params.num_robots>1 else draw_gradient(robots[0].U)
    for i, robot in enumerate(robots): 
        plt.plot(robot.sp[0], robot.sp[1], marker=smiley, color='blue', markersize=10, zorder=15) # robots poses
        plt.annotate(str(i), xy=(robot.sp[0], robot.sp[1]), xytext=(robot.sp[0], robot.sp[1]))

        draw_circle(robot, 'red')
        # cal_perceptual_robustness(robot)
        # # 计算圆上的点的坐标
        # x = np.cos(theta) * params.influence_radius + robot.sp[0]
        # y = np.sin(theta) * params.influence_radius + robot.sp[1]

        # # 绘制圆
        # # plt.figure(figsize=(6, 6))
        # plt.plot(x, y, color='blue')
        # plt.fill(x, y, color='green', alpha=0.3)  # 填充圆的内部
        # # plt.xlim(-1.1, 1.1)
        # # plt.ylim(-1.1, 1.1)
        # plt.gca().set_aspect('equal', adjustable='box')
    
    plt.plot(attack_robot.sp[0], attack_robot.sp[1], 'o', color='red', markersize=1, zorder=15)
    x = np.cos(theta) * params.drone_vel * 0.01 + attack_robot.sp[0]
    y = np.sin(theta) * params.drone_vel * 0.01 + attack_robot.sp[1]
    plt.plot(x, y, color='blue')
    plt.fill(x, y, color='green', alpha=0.3)
    plt.gca().set_aspect('equal', adjustable='box')

    # robots_poses = []
    # for robot in robots: robots_poses.append(robot.sp)
    # robots_poses.sort(key=lambda p: atan2(p[1]-centroid[1],p[0]-centroid[0]))
    # plt.gca().add_patch( Polygon(robots_poses, color='yellow') )
    # plt.plot(centroid[0], centroid[1], '*', color='b', markersize=10) # label='Centroid position'
    # plt.plot(robot1.route[:,0], robot1.route[:,1], linewidth=2, color='green', label="Leader's path", zorder=10)
    # for robot in robots[1:]: plt.plot(robot.route[:,0], robot.route[:,1], '--', linewidth=2, color='red', zorder=10)
    plt.plot(P[:,0], P[:,1], linewidth=3, color='orange') # label='Global planner path'
    plt.plot(traj_global[sp_ind,0], traj_global[sp_ind,1], 'ro', color='blue', markersize=7) # label='Global planner setpoint'
    plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20) # label='start'
    plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20) # label='goal'

    if is_crash:
        plt.text(0, 0, "无人机发生碰撞", fontdict=None, withdash=False)
    
    # ------------------------------------------------------
    # for robot in metrics.robots:
    #     plt.plot(robot.route[:,0], robot.route[:,1], '--', label='drone %d' %robot.id, linewidth=2)
    # ------------------------------------------------------
    
    # plt.legend()

# Initialization
init_fonts(small=12, medium=16, big=26)
params = Params()
xy_start = np.array([1.2, 1.0])
xy_goal =  np.array([-1.5, -1.4])
# xy_goal =  np.array([1.3, 1.0])

# Obstacles map construction
obstacles = [
    # bugtrap
    np.array([[0.5, 0], [2.5, 0.], [2.5, 0.3], [0.5, 0.3]]),
    np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]),
    # np.array([[0.5, 1.5], [1.5, 1.5], [1.5, 1.8], [0.5, 1.8]]),
    # angle
    np.array([[-2, -2], [-0.5, -2], [-0.5, -1.8], [-2, -1.8]]),
    np.array([[-0.7, -1.8], [-0.5, -1.8], [-0.5, -0.8], [-0.7, -0.8]]),

    # ----------------------------------------------------------------

    # np.array([[-2.47, 2.47], [-2, 2.47], [-2, 2], [-2.47, 2]]),
    # np.array([[-1.47, 2.47], [-1, 2.47], [-1, 2], [-1.47, 2]]),
    # np.array([[-0.47, 2.47], [-0, 2.47], [-0, 2], [-0.47, 2]]),
    # np.array([[0.53, 2.47], [1, 2.47], [1, 2], [0.53, 2]]),
    # np.array([[1.53, 2.47], [2, 2.47], [2, 2], [1.53, 2]]),

    # np.array([[-2.47, 1.47], [-2, 1.47], [-2, 1], [-2.47, 1]]),
    # np.array([[-1.47, 1.47], [-1, 1.47], [-1, 1], [-1.47, 1]]),
    # np.array([[-0.47, 1.47], [0, 1.47], [0., 1], [-0.47, 1]]),
    # np.array([[0.53, 1.47], [1, 1.47], [1., 1], [0.53, 1]]),
    # np.array([[1.53, 1.47], [2, 1.47], [2., 1], [1.53, 1]]),

    # np.array([[-2.47, 0.47], [-2, 0.47], [-2, 0], [-2.47, 0]]),
    # np.array([[-1.47, 0.47], [-1, 0.47], [-1, 0], [-1.47, 0]]),
    # np.array([[-0.47, 0.47], [-0, 0.47], [-0, 0], [-0.47, 0]]),
    # np.array([[0.53, 0.47], [1, 0.47], [1, 0], [0.53, 0]]),
    # np.array([[1.53, 0.47], [2, 0.47], [2, 0], [1.53, 0]]),

    # np.array([[-2.47, -0.53], [-2, -0.53], [-2, -1], [-2.47, -1]]),
    # np.array([[-1.47, -0.53], [-1, -0.53], [-1, -1], [-1.47, -1]]),
    # np.array([[-0.47, -0.53], [-0, -0.53], [-0, -1], [-0.47, -1]]),
    # np.array([[0.53, -0.53], [1, -0.53], [1, -1], [0.53, -1]]),
    # np.array([[1.53, -0.53], [2, -0.53], [2, -1], [1.53, -1]]),

    # np.array([[-2.47, -1.53], [-2, -1.53], [-2, -2], [-2.47, -2]]),
    # np.array([[-1.47, -1.53], [-1, -1.53], [-1, -2], [-1.47, -2]]),
    # np.array([[-0.47, -1.53], [-0, -1.53], [-0, -2], [-0.47, -2]]),
    # np.array([[0.53, -1.53], [1, -1.53], [1, -2], [0.53, -2]]),
    # np.array([[1.53, -1.53], [2, -1.53], [2, -2], [1.53, -2]]),

    # ----------------------------------------------------------------

    # walls
    np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]), # comment this for better 3D visualization
    np.array([[-2.5, 2.47], [2.5, 2.47], [2.5, 2.5], [-2.5, 2.5]]),
    np.array([[-2.5, -2.47], [-2.47, -2.47], [-2.47, 2.47], [-2.5, 2.47]]),
    np.array([[2.47, -2.47], [2.5, -2.47], [2.5, 2.47], [2.47, 2.47]]), # comment this for better 3D visualization

    #   # moving obstacle
    #   np.array([[-2.3, 2.0], [-2.2, 2.0], [-2.2, 2.1], [-2.3, 2.1]]),
    #   np.array([[2.3, -2.3], [2.4, -2.3], [2.4, -2.2], [2.3, -2.2]]),
    #   np.array([[0.0, -2.3], [0.1, -2.3], [0.1, -2.2], [0.0, -2.2]]),
]
"""" Narrow passage """
# passage_width = 0.3
# passage_location = 0.0
# obstacles = [
#     # narrow passage
#         np.array([[-2.5, -0.5], [-passage_location-passage_width/2., -0.5], [-passage_location-passage_width/2., 0.5], [-2.5, 0.5]]),
#         np.array([[-passage_location+passage_width/2., -0.5], [2.5, -0.5], [2.5, 0.5], [-passage_location+passage_width/2., 0.5]]),
#     ]
# obstacles = []

robots = []
for i in range(params.num_robots):
    robots.append(Robot(i+1))
robot1 = robots[0]
robot1.leader=True

attack_robot = Robot(' attack')
attack_robot.sp = np.array([0.0, 2.0])

# Metrics to measure (for postprocessing)
class Metrics:
    def __init__(self):
        self.mean_dists_array = []
        self.max_dists_array = []
        self.centroid_path = [np.array([0,0])]
        self.centroid_path_length = 0
        self.robots = []
        self.vels_mean = []
        self.vels_max = []
        self.area_array = []
        self.cpu_usage_array = [] # [%]
        self.memory_usage_array = [] # [MiB]

        self.folder_to_save = '/home/rus/Desktop/'

metrics = Metrics()

def rob_map(rbt: Robot, nrows=500, ncols=500) -> tuple:
    global grid

    min_curve_dis = (rbt.curve_count - 1) / rbt.curve_count * params.influence_radius

    min_rob_pos = [0, 0]
    min_rob = math.inf
    
    # 对每个位置点计算健壮性
    for i in range(nrows):
        for j in range(ncols):
            # 计算位置点到无人机位置的距离
            dis = norm(grid2meters([i, j]) - rbt.sp)
            if dis > params.influence_radius:
                # 位置点在无人机感知范围外
                grid[i][j] += rbt.no_influence_rob
            elif dis >= min_curve_dis:
                # 位置点在无人机感知圆环范围内
                min_dis_pos = 0
                min_dis = math.inf

                # 计算位置点到各个感知点的距离，最短距离所在的位置点就是代表的位置点
                for p in range(len(rbt.perceptual_robustness_list)):
                    perceptual_dis = norm(grid2meters([i, j]) - rbt.perceptual_robustness_list[p][0: 2])
                    if perceptual_dis < min_dis:
                        min_dis = perceptual_dis
                        min_dis_pos = p

                grid[i][j] += rbt.perceptual_robustness_list[min_dis_pos][2]
                # print(grid2meters([i, j]), grid[i][j])
                # grid[i][j] = min_dis_pos + 1
            else:
                # 位置点在无人机感知圆环范围外，攻击机下一步不可能到达该位置
                grid[i][j] += math.inf
            
            if grid[i][j] < min_rob:
                min_rob = grid[i][j]
                min_rob_pos = grid2meters([i, j])

    return min_rob, min_rob_pos

def cal_perceptual_robustness(rbt: Robot, traj_global, sp_ind, radius):
    """计算无人机感知范围内的健壮性

    Args:
        rbt (Robot): 无人机
    """
    # 保存位置信息
    rbt_sp = rbt.sp
    rbt_route = rbt.route
    rbt.sp_global = traj_global[sp_ind,:]

    # 周围感知健壮性数据清除
    rbt.perceptual_robustness_list.clear()

    # 数据初始化
    # r = params.influence_radius
    r = radius
    count = rbt.curve_count
    count2 = rbt.straight_count
    x_c, y_c = rbt.sp[0], rbt.sp[1]

    # 计算周围感知点
    sub_r = r / count * ((2 * (count - 1) + 1) / 2)
    for j in range(count2):
        x = np.cos(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + x_c
        y = np.sin(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + y_c
        # plt.plot(x, y, color=color, marker='o')
        rbt.perceptual_robustness_list.append([x, y])

    # 从每个周围点开始计算健壮性
    rbt.monitor.logger.debug("开始计算周围感知健壮性")
    for i in range(len(rbt.perceptual_robustness_list)):
        point_obs = poses2polygons(rbt.perceptual_robustness_list[i])
        rbt.local_planner(np.array(obstacles + point_obs), params)

        rob = rbt.monitor.multi_rule_monitor(get_info(rbt))
        rbt.monitor.logger.debug("周围点%s的健壮性为%s" % (rbt.perceptual_robustness_list[i], rob))
        rbt.perceptual_robustness_list[i].append(rob)

        # 无人机数据还原
        rbt.sp = rbt_sp
        rbt.route = rbt_route

def cal_attack_perceptual_robustness(rbt: Robot, traj_global, sp_ind, radius):

    # 保存位置信息
    rbt_sp = rbt.sp
    rbt_route = rbt.route
    store_data = list()
    for rob in robots:
        store_data.append([rob.sp, rob.route])

    rbt.perceptual_robustness_list.clear()

    robot1.sp_global = traj_global[sp_ind,:]

    r = radius
    count = rbt.curve_count
    count2 = rbt.straight_count
    x_c, y_c = rbt.sp[0], rbt.sp[1]

    sub_r = r / count * ((2 * (count - 1) + 1) / 2)
    for j in range(count2):
        x = np.cos(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + x_c
        y = np.sin(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + y_c
        # plt.plot(x, y, color=color, marker='o')
        rbt.perceptual_robustness_list.append([x, y])

    rbt.monitor.logger.debug("开始计算周围感知健壮性")
    for i in range(len(rbt.perceptual_robustness_list)):
        rbt.sp_global = rbt.perceptual_robustness_list[i]
        rbt.local_planner(obstacles, params)

        if uav_carsh_check(rbt):
            rbt.perceptual_robustness_list[i].append(math.inf)
            rbt.sp = rbt_sp
            rbt.route = rbt_route
            continue

        rbt_obs = poses2polygons([rbt.sp])
        robot1.local_planner(np.array(obstacles + rbt_obs), params)

        followers_sp_global = formation(params.num_robots, robot1.sp_global, v=normalize(robot1.sp_global-robot1.sp), l=params.interrobots_dist)
        for j in range(len(followers_sp_global)): robots[j+1].sp_global = followers_sp_global[j]
        for p in range(len(followers_sp)): # formation poses correction with local planner
            # robots repel from each other inside the formation
            robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [robot1.sp, rbt.sp]) if k!=p] # all poses except the robot[p]
            robots_obstacles = poses2polygons(robots_obstacles_sp) # each drone is defined as a small cube for inter-robots collision avoidance
            obstacles1 = np.array(obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
            # follower robot's position correction with local planner
            robots[p+1].local_planner(obstacles1, params)
            # followers_sp[p] = robots[p+1].sp

        total_rob = 0.0
        for rob in robots:
            total_rob += rob.monitor.multi_rule_monitor(get_info(rob))
        rbt.monitor.logger.debug("攻击机攻击方向%s导致无人机蜂群健壮性为：%s" % (str(rbt.perceptual_robustness_list[i]), total_rob))
        rbt.perceptual_robustness_list[i].append(total_rob)

        # 无人机数据还原
        rbt.sp = rbt_sp
        rbt.route = rbt_route
        for x in range(len(robots)):
            robots[x].sp = store_data[x][0]
            robots[x].route = store_data[x][1]

def draw_circle(rbt: Robot, color='blue'):
    x_c, y_c = rbt.sp[0], rbt.sp[1]

    theta = np.linspace(0, 2*np.pi, 1000)
    r = params.influence_radius

    # 计算圆上的点的坐标
    x = np.cos(theta) * r + x_c
    y = np.sin(theta) * r + y_c

    # 绘制圆
    # plt.figure(figsize=(6, 6))
    plt.plot(x, y, color=color)
    plt.fill(x, y, color='green', alpha=0.3)  # 填充圆的内部

    count = 5

    # for i in range(count - 1):
    #     x = np.cos(theta) * (i+1) * r / count + x_c
    #     y = np.sin(theta) * (i+1) * r / count + y_c
    #     plt.plot(x, y, color=color)
    #     plt.fill(x, y, color='green', alpha=0.3)

    x = np.cos(theta) * (count-1) * r / count + x_c
    y = np.sin(theta) * (count-1) * r / count + y_c
    plt.plot(x, y, color=color)
    plt.fill(x, y, color='green', alpha=0.3)

    count2 = 5

    for i in range(count2):
        x = np.cos(2 * np.pi / count2 * i) * r + x_c
        y = np.sin(2 * np.pi / count2 * i) * r + y_c
        plt.plot([x_c, x], [y_c, y], color=color)

    # for i in range(count):
    #     sub_r = r / count * ((2 * i + 1) / 2)
    #     for j in range(count2):
    #         x = np.cos(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + x_c
    #         y = np.sin(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + y_c
    #         plt.plot(x, y, color=color, marker='o')

    # sub_r = r / count * ((2 * (count - 1) + 1) / 2)
    # for j in range(count2):
    #     x = np.cos(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + x_c
    #     y = np.sin(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + y_c
    #     plt.plot(x, y, color=color, marker='o')
        # rbt.perceptual_robustness_list.append([x, y])

    for point in rbt.perceptual_robustness_list:
        plt.plot(point[0], point[1], color=color, marker='o')

    plt.gca().set_aspect('equal', adjustable='box')

def attack(traj_global, sp_ind, followers_sp):

    global is_crash

    # 发动攻击尝试前存储所有无人机的位置信息
    store_data = list()
    for rob in robots:
        store_data.append((rob.sp, rob.route))
    store_data.append((attack_robot.sp, attack_robot.route))

    attack_robot.monitor.logger.debug("开始计算攻击机不移动干扰下的集群健壮性")
    attack_rob_obs = poses2polygons([attack_robot.sp])
    robot1.sp_global = traj_global[sp_ind,:]
    robot1.local_planner(np.array(obstacles + attack_rob_obs), params)

    followers_sp_global = formation(params.num_robots, robot1.sp_global, v=normalize(robot1.sp_global-robot1.sp), l=params.interrobots_dist)
    for i in range(len(followers_sp_global)): robots[i+1].sp_global = followers_sp_global[i]
    for p in range(len(followers_sp)): 
        robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [robot1.sp, attack_robot.sp]) if i!=p]
        robots_obstacles = poses2polygons( robots_obstacles_sp )
        obstacles1 = np.array(obstacles + robots_obstacles)
        robots[p+1].local_planner(obstacles1, params)
        # followers_sp[p] = robots[p+1].sp
    
    normal_rob = 0.0
    for rob in robots:
        normal_rob += rob.monitor.multi_rule_monitor(get_info(rob))
    attack_robot.monitor.logger.debug("计算结束，无人机集群在攻击机不移动情况下的健壮性为%s" % (normal_rob))
    
    min_rob = normal_rob
    min_rob_global_sp = np.array([0, 0])
    attack_robot.monitor.logger.debug("攻击机开始进行攻击试探")

    robot_obs_sp = followers_sp + [robot1.sp]
    robots_obs = poses2polygons(robot_obs_sp)
    obs_total = np.array(obstacles + robots_obs)

    for i in range(20):
        
        # 进行无人机位置信息还原
        for x in range(len(robots)):
            robots[x].sp = store_data[x][0]
            robots[x].route = store_data[x][1]

        attack_robot.sp = store_data[-1][0]
        attack_robot.route = store_data[-1][1]
        robot1.sp_global = traj_global[sp_ind,:]

        # TODO 修改攻击策略
        # 随机产生一个方向
        theta = random.random() * 2 * math.pi
        r = random.uniform(0, 5)
        x, y = math.sin(theta) * (r ** 0.5), math.cos(theta) * (r ** 0.5)
        attack_robot.sp_global = np.array([x, y])
        attack_robot.monitor.logger.debug("随机产生攻击方向%s: %s" % (i, attack_robot.sp_global))
        # attack_robot.local_planner(obs_total, params)

        # attack_direction_vector = attack_robot.sp_global - attack_robot.sp
        # attack_robot.sp = attack_robot.sp + 0.01 * params.drone_vel * (attack_direction_vector / norm(attack_direction_vector))
        # attack_robot.route = np.vstack( [attack_robot.route, attack_robot.sp] )

        attack_robot.local_planner(obstacles, params)

        if uav_carsh_check(attack_robot):
            continue

        attack_rob_obs = poses2polygons([attack_robot.sp])
        robot1.local_planner(np.array(obstacles + attack_rob_obs), params)

        followers_sp_global = formation(params.num_robots, robot1.sp_global, v=normalize(robot1.sp_global-robot1.sp), l=params.interrobots_dist)
        for j in range(len(followers_sp_global)): robots[j+1].sp_global = followers_sp_global[j]
        for p in range(len(followers_sp)): # formation poses correction with local planner
            # robots repel from each other inside the formation
            robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [robot1.sp, attack_robot.sp]) if k!=p] # all poses except the robot[p]
            robots_obstacles = poses2polygons(robots_obstacles_sp) # each drone is defined as a small cube for inter-robots collision avoidance
            obstacles1 = np.array(obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
            # follower robot's position correction with local planner
            robots[p+1].local_planner(obstacles1, params)
            # followers_sp[p] = robots[p+1].sp
        
        total_rob = 0.0
        for rob in robots:
            total_rob += rob.monitor.multi_rule_monitor(get_info(rob))
        attack_robot.monitor.logger.debug("随机攻击方向%s导致最终无人机蜂群健壮性为: %s" % (i, total_rob))

        if total_rob < min_rob:
            min_rob = total_rob
            min_rob_global_sp = attack_robot.sp_global
    
    if min_rob == -math.inf:
        is_crash = True
    
    # 进行无人机位置信息还原
    for x in range(len(robots)):
        robots[x].sp = store_data[x][0]
        robots[x].route = store_data[x][1]

    attack_robot.sp = store_data[-1][0]
    attack_robot.route = store_data[-1][1]
    robot1.sp_global = traj_global[sp_ind,:]
    
    if min_rob >= normal_rob:
        attack_robot.monitor.logger.debug("随机产生的攻击未对集群造成负面影响，攻击机直接飞向领头机")
        attack_robot.sp_global = robot1.sp
        # attack_robot.local_planner(obs_total, params)

        # attack_direction_vector = attack_robot.sp_global - attack_robot.sp
        # attack_robot.sp = attack_robot.sp + 0.01 *  params.drone_vel * (attack_direction_vector / norm(attack_direction_vector))
        # attack_robot.route = np.vstack( [attack_robot.route, attack_robot.sp] )

        attack_robot.local_planner(obstacles, params)

        attack_rob_obs = poses2polygons([attack_robot.sp])
        robot1.local_planner(np.array(obstacles + attack_rob_obs), params)

        followers_sp_global = formation(params.num_robots, robot1.sp_global, v=normalize(robot1.sp_global-robot1.sp), l=params.interrobots_dist)
        for j in range(len(followers_sp_global)): robots[j+1].sp_global = followers_sp_global[j]
        for p in range(len(followers_sp)): 
            robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [robot1.sp, attack_robot.sp]) if k!=p]
            robots_obstacles = poses2polygons( robots_obstacles_sp )
            obstacles1 = np.array(obstacles + robots_obstacles)
            robots[p+1].local_planner(obstacles1, params)
            followers_sp[p] = robots[p+1].sp
        return
    
    attack_robot.monitor.logger.debug("攻击机攻击试探结束，最终确定攻击方向为: %s，将导致无人机集群健壮性变成%s" % (min_rob_global_sp, min_rob))
    attack_robot.sp_global = min_rob_global_sp
    # attack_robot.local_planner(obs_total, params)

    # attack_direction_vector = attack_robot.sp_global - attack_robot.sp
    # attack_robot.sp = attack_robot.sp + 0.01 * params.drone_vel * (attack_direction_vector / norm(attack_direction_vector))
    # attack_robot.route = np.vstack( [attack_robot.route, attack_robot.sp] )

    attack_robot.local_planner(obstacles, params)

    attack_rob_obs = poses2polygons([attack_robot.sp])
    robot1.local_planner(np.array(obstacles + attack_rob_obs), params)

    followers_sp_global = formation(params.num_robots, robot1.sp_global, v=normalize(robot1.sp_global-robot1.sp), l=params.interrobots_dist)
    for i in range(len(followers_sp_global)): robots[i+1].sp_global = followers_sp_global[i]
    for p in range(len(followers_sp)): # formation poses correction with local planner
        # robots repel from each other inside the formation
        robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [robot1.sp, attack_robot.sp]) if i!=p] # all poses except the robot[p]
        robots_obstacles = poses2polygons(robots_obstacles_sp) # each drone is defined as a small cube for inter-robots collision avoidance
        obstacles1 = np.array(obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
        # follower robot's position correction with local planner
        robots[p+1].local_planner(obstacles1, params)
        followers_sp[p] = robots[p+1].sp

def attack2(traj_global, sp_ind):

    # 发动攻击尝试前存储所有无人机的位置信息
    # store_data = list()
    # for rob in robots:
    #     store_data.append((rob.sp, rob.route))
    # store_data.append((attack_robot.sp, attack_robot.route))

    global grid, nrows, ncols, is_crash
    grid = np.zeros((nrows, ncols))

    # 每次发送攻击前要计算无人机当前位置不受影响的健壮性
    for rob in robots:
        rob.cal_no_influence_rob()

    min_rob = math.inf
    min_rob_pos = [0, 0]

    # 计算攻击机周围感知健壮性
    cal_attack_perceptual_robustness(attack_robot, traj_global, sp_ind, 0.01 * params.drone_vel)
    attack_robot.monitor.logger.debug("攻击机的周围感知健壮性为%s" % (str(attack_robot.perceptual_robustness_list)))

    # 对每个无人机都计算周围感知健壮性
    for rbt in robots:
        cal_perceptual_robustness(rbt, traj_global, sp_ind, params.influence_radius)
        min_rob, min_rob_pos = rob_map(rbt)
    
    for i in range(len(attack_robot.perceptual_robustness_list)):
        if attack_robot.perceptual_robustness_list[i][2] < min_rob:
            min_rob = attack_robot.perceptual_robustness_list[i][2]
            min_rob_pos = attack_robot.perceptual_robustness_list[i][0: 2]
    
    if min_rob == -math.inf:
        is_crash = True
    
    # 发动攻击
    attack_robot.monitor.logger.debug("最终确定无人机攻击方向为%s，产生的健壮性为%s" % (str(min_rob_pos), min_rob))
    attack_robot.sp_global = min_rob_pos
    attack_robot.local_planner(obstacles, params)

    attack_rob_obs = poses2polygons([attack_robot.sp])
    robot1.local_planner(np.array(obstacles + attack_rob_obs), params)

    followers_sp_global = formation(params.num_robots, robot1.sp_global, v=normalize(robot1.sp_global-robot1.sp), l=params.interrobots_dist)
    for j in range(len(followers_sp_global)): robots[j+1].sp_global = followers_sp_global[j]
    for p in range(len(followers_sp)): 
        robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [robot1.sp, attack_robot.sp]) if k!=p]
        robots_obstacles = poses2polygons( robots_obstacles_sp )
        obstacles1 = np.array(obstacles + robots_obstacles)
        robots[p+1].local_planner(obstacles1, params)
        followers_sp[p] = robots[p+1].sp
    # return

    # with open("data_collection/d.txt", 'a') as f:
    #     for row in grid:
    #         for col in row:
    #             # if col == math.inf:
    #             #     f.write(str(col) + ". ")
    #             # else:
    #             #     if len(str(col)) < 4:
    #             #         f.write(str(col) + "0 ")
    #             #     else:
    #             #         f.write(str(round(col, 2)) + " ")
    #             f.write(str(col) + " ")
    #         f.write("\n")

def uav_carsh_check(attack_robot: Robot) -> bool:
    for rbt in robots:
        D = math.sqrt(math.pow((attack_robot.sp[0] - rbt.sp[0]), 2) + math.pow((attack_robot.sp[1] - rbt.sp[1]), 2))
        if D < 0.07:
            return True
    return False

# Layered Motion Planning: RRT (global) + Potential Field (local)
if __name__ == '__main__':
    fig2D = plt.figure(figsize=(10,10))
    draw_map(obstacles)
    plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20, label='start')
    plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20, label='goal')

    P_long = rrt_path(obstacles, xy_start, xy_goal, params)
    print('Path Shortenning...')
    P = ShortenPath(P_long, obstacles, smoothiters=50) # P = [[xN, yN], ..., [x1, y1], [x0, y0]]
    # P = P_path

    traj_global = waypts2setpts(P, params)
    # traj_global = np.array(traj_global_path)
    P = np.vstack([P, xy_start])
    plt.plot(P[:,0], P[:,1], linewidth=3, color='orange', label='Global planner path')
    plt.pause(0.5)

    # with open('data_collection/path2.txt', 'w') as f:
    #     f.write(str(P) + "/n")
    #     f.write(str(traj_global))
    sp_ind = 0
    robot1.route = np.array([traj_global[0,:]])
    robot1.sp = robot1.route[-1,:]

    followers_sp = formation(params.num_robots, leader_des=robot1.sp, v=np.array([0,-1]), l=params.interrobots_dist)
    for i in range(len(followers_sp)):
        robots[i+1].sp = followers_sp[i]
        robots[i+1].route = np.array([followers_sp[i]])
    for rbt in robots:
        print(rbt.sp)
    print('Start movement...')
    t0 = time.time(); 
    # t_array = []

    while True: # loop through all the setpoint from global planner trajectory, traj_global
        t_array.append( time.time() - t0 )
        # print("Current time [sec]: ", time.time() - t0)

        if is_crash:
            print('无人机发生碰撞')
            break

        dist_to_goal = norm(robot1.sp - xy_goal)
        if dist_to_goal < params.goal_tolerance: # [m]
            print('Goal is reached')
            break
        if len(obstacles)>2: obstacles = move_obstacles(obstacles, params) # change poses of some obstacles on the map

        attack(traj_global, sp_ind, followers_sp)
        attack2(traj_global, sp_ind)

        # # leader's setpoint from global planner
        # robot1.sp_global = traj_global[sp_ind,:]
        # # correct leader's pose with local planner
        # robot1.local_planner(obstacles, params)
        # # robot1.monitor.multi_rule_monitor(get_info(robot1))

        # """ adding following robots in the swarm """
        # # formation poses from global planner
        # followers_sp_global = formation(params.num_robots, robot1.sp_global, v=normalize(robot1.sp_global-robot1.sp), l=params.interrobots_dist)
        # for i in range(len(followers_sp_global)): robots[i+1].sp_global = followers_sp_global[i]
        # for p in range(len(followers_sp)): # formation poses correction with local planner
        #     # robots repel from each other inside the formation
        #     robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [robot1.sp]) if i!=p] # all poses except the robot[p]
        #     robots_obstacles = poses2polygons( robots_obstacles_sp ) # each drone is defined as a small cube for inter-robots collision avoidance
        #     obstacles1 = np.array(obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
        #     # follower robot's position correction with local planner
        #     robots[p+1].local_planner(obstacles1, params)
        #     followers_sp[p] = robots[p+1].sp
        #     # robots[p+1].monitor.multi_rule_monitor(get_info(robots[p+1]))

        # centroid pose:
        centroid = 0
        for robot in robots: centroid += robot.sp / len(robots)
        metrics.centroid_path = np.vstack([metrics.centroid_path, centroid])
        # dists to robots from the centroid:
        dists = []
        for robot in robots:
            dists.append( norm(centroid-robot.sp) )
        # Formation size estimation
        metrics.mean_dists_array.append(np.mean(dists)) # Formation mean Radius
        metrics.max_dists_array.append(np.max(dists)) # Formation max Radius

        # Algorithm performance (CPU and memory usage)
        metrics.cpu_usage_array.append( cpu_usage() )
        metrics.memory_usage_array.append( memory_usage() )
        # print("CPU: ", cpu_usage())
        # print("Memory: ", memory_usage())

        # visualization
        if params.visualize:
            plt.cla()
            visualize2D()

            plt.draw()
            plt.pause(0.01)

        # update loop variable
        if sp_ind < traj_global.shape[0]-1 and norm(robot1.sp_global - centroid) < params.max_sp_dist: sp_ind += 1

        """ Flight data postprocessing """
    if params.postprocessing:
        t_array = t_array[1:]
        metrics.t_array = t_array
        metrics.centroid_path = metrics.centroid_path[1:,:]
        metrics.centroid_path_length = path_length(metrics.centroid_path)
        for robot in robots: metrics.robots.append( robot )

        postprocessing(metrics, params, visualize=1)
        if params.savedata: save_data(metrics)

    # close windows if Enter-button is pressed
    plt.draw()
    plt.pause(0.1)
    input('Hit Enter to close')
    plt.close('all')