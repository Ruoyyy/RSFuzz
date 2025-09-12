import sys,os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))
from adaptive_swarm.scripts.layered_planner.katz import Graph
import numpy as np
from numpy.linalg import norm
from adaptive_swarm.scripts.layered_planner.tools import poses2polygons, init_fonts, draw_map, waypts2setpts, formation, normalize, cpu_usage, memory_usage, path_length, postprocessing, save_data
from tool.tool import Monitor, nearest_obs
from adaptive_swarm.scripts.layered_planner.potential_fields import grid_map, combined_potential, meters2grid, grid2meters
import matplotlib.pyplot as plt
from adaptive_swarm.scripts.layered_planner.rrt import rrt_path, ShortenPath
import time
import random
import math
from svgpath2mpl import parse_path
from time import strftime
from os import mkdir
# from data_collection.path import P_list, traj_global_list
from copy import deepcopy
from threading import Thread

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
        self.influence_radius = 0.10 # [m] potential fields radius, defining repulsive area size near the obstacle
        self.goal_tolerance = 0.05 # [m], maximum distance threshold to reach the goal
        self.num_robots = 4 # number of robots in the formation
        self.interrobots_dist = 0.3 # [m], distance between robots in default formation
        self.max_sp_dist = 0.2 * self.drone_vel# * np.sqrt(self.num_robots) # [m], maximum distance between current robot's pose and the sp from global planner
        self.dir = 0

class Robot:
    def __init__(self, id, monitor=None, create_time=time.localtime):
        self.id = id
        self.sp = np.array([0, 0])
        self.sp_global = np.array([0,0])
        self.route = np.array([self.sp])
        self.vel_array = []
        self.U_a = 0 # attractive APF function
        self.U_r = 0 # repulsive APF function
        self.U = 0 # total APF function
        self.leader = False
        self.perceptual_robustness_list: list[list] = []
        self.min_perceptual_robustness = math.inf
        self.min_perceptual_robustness_pos = [0, 0]
        self.curve_count = 5
        self.straight_count = 8
        self.no_influence_rob = 0.0
        if not monitor:
            self.monitor = Monitor('uav' + str(self.id), create_time=create_time)
        else:
            self.monitor = monitor

    def local_planner(self, obstacles, params: Params):
        """
        This function computes the next_point
        given current location (self.sp) and potential filed function, f.
        It also computes mean velocity, V, of the gradient map in current point.
        """
        obstacles_grid = grid_map(obstacles)
        self.U, self.U_a, self.U_r = combined_potential(obstacles_grid, self.sp_global, params.influence_radius)
        [gy, gx] = np.gradient(-self.U)
        iy, ix = np.array(meters2grid(self.sp), dtype=int )
        w = 20 # smoothing window size for gradient-velocity
        ax = np.mean(gx[ix-int(w/2) : ix+int(w/2), iy-int(w/2) : iy+int(w/2)])
        ay = np.mean(gy[ix-int(w/2) : ix+int(w/2), iy-int(w/2) : iy+int(w/2)])
        # ax = gx[ix, iy]; ay = gy[ix, iy]
        self.V = params.drone_vel * np.array([ax, ay])
        self.vel_array.append(norm(self.V))
        dt = 0.01 * params.drone_vel / norm([ax, ay]) if norm([ax, ay])!=0 else 0.01
        # self.sp += dt**2/2. * np.array( [ax, ay] )
        self.sp = self.sp + dt * np.array( [ax, ay] ) #+ 0.1*dt**2/2. * np.array( [ax, ay] )
        self.route = np.vstack( [self.route, self.sp] )

    def cal_no_influence_rob(self, obstacles, params: Params):
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

                info = dict()
                info['uav_pos'] = self.sp
                info['obs_list'] = obstacles[:-4]
                info['DIS_PRE_MIN_OBS'] = params.influence_radius
                info['time'] = 0
                self.no_influence_rob = self.monitor.rule1_monitor(info)

                self.monitor.logger.debug("无人机感知范围外的健壮性为%s" % self.no_influence_rob)

                # 还原位置
                self.sp = rbt_sp
                self.sp_global = rbt_route
                return 

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
        self.t_array = None

        self.folder_to_save = '/home/ruoyu/Desktop/'

class attack():
    def __init__(self, attack_strategy=5) -> None:
        self.start_time = time.time()
        self.t_array = [0] # 时间列表
        self.attack_time = time.localtime() # 开始攻击时间

        self.res_to_save_folder_path = 'data_collection/attack_res/' # 数据存储文件夹路径
        self.attack_floder_name = strftime("%Y-%m-%d-%H-%M-%S", self.attack_time)
        self.attack_floder_path = self.res_to_save_folder_path + self.attack_floder_name + '/' # 单次攻击存储文件夹路径
        mkdir(self.attack_floder_path) # 创建文件夹
        self.res_to_save_file_path = self.attack_floder_path + 'res.txt' # 测试结果存储文件路径
        self.res_pic_to_save_file_path = self.attack_floder_path + 'res.jpg' # 测试结果存储图片路径
        self.attack_point_to_save_file_path = self.attack_floder_path + 'attack_path.txt' # 攻击点存储文件路径
        self.attack_point_pic_to_save_file_path = self.attack_floder_path + 'attack_point_pic.jpg' # 攻击点存储图片路径
        self.attack_path_pic_to_save_file_path = self.attack_floder_path + 'attack_path_pic.jpg' # 攻击路径存储图片路径
        self.attack_path_data_to_save_file_path = self.attack_floder_path + 'attack_path_data.txt' # 攻击路径存储数据路径
        self.rob_change_pic_to_save_file_path = self.attack_floder_path + 'rob_change_pic.jpg' # 健壮性变化趋势存储图片路径

        # 将地图转换成矩阵
        self.nrows = 500 # 矩阵行数
        self.ncols = 500 # 矩阵列表
        self.grid = np.zeros((self.nrows, self.ncols)) # 地图矩阵
        
        # self.is_crash = False # 检测是否碰撞参数

        # 初始化字体
        init_fonts(small=12, medium=16, big=26)

        self.params = Params() # 初始化参数
        self.xy_start = np.array([1.2, 1.25]) # 起点
        self.xy_goal =  np.array([2.2, -2.2]) # 终点

        # self.xy_start = np.array([1.2, 1.25])
        # self.xy_goal =  np.array([2.2, -2.2])

        # self.obstacles = [
        #     # 障碍物
        #     np.array([[0.5, 0], [2.5, 0.], [2.5, 0.3], [0.5, 0.3]]),
        #     np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]),
        #     np.array([[-2, -2], [-0.5, -2], [-0.5, -1.8], [-2, -1.8]]),
        #     np.array([[-0.7, -1.8], [-0.5, -1.8], [-0.5, -0.8], [-0.7, -0.8]]),

        #     # 墙壁
        #     np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),
        #     np.array([[-2.5, 2.47], [2.5, 2.47], [2.5, 2.5], [-2.5, 2.5]]),
        #     np.array([[-2.5, -2.47], [-2.47, -2.47], [-2.47, 2.47], [-2.5, 2.47]]),
        #     np.array([[2.47, -2.47], [2.5, -2.47], [2.5, 2.47], [2.47, 2.47]]),
        # ]

        self.obstacles = [
            # wall
            np.array([[-1.0, 0], [2.5, 0.], [2.5, 0.3], [-1.0, 0.3]]),
            # np.array([[0.5, 1.6], [0.8, 1.6], [0.8, 2.5], [0.5, 2.5]]),
            np.array([[0.5, 2.0], [0.8, 2.0], [0.8, 2.5], [0.5, 2.5]]),
            np.array([[0.0, -2.5], [0.3, -2.5], [0.3, -1.6], [0.0, -1.6]]),
            np.array([[-1.3, 0.3], [-0.7, 0.3], [-0.7, 0.6], [-1.3, 0.6]]),
            np.array([[-1.0, 1.8], [-0.7, 1.8], [-0.7, 2.5], [-1.0, 2.5]]),
            np.array([[-2.0, -2.47], [-1.4, -2.47], [-1.4, -1.2], [-2.0, -1.2]]),
            # np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),
            # np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),
            # np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),
            # np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),
            # np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),
            # np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),

            # room
            np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),
            np.array([[-2.5, 2.47], [2.5, 2.47], [2.5, 2.5], [-2.5, 2.5]]),
            np.array([[-2.5, -2.47], [-2.47, -2.47], [-2.47, 2.47], [-2.5, 2.47]]),
            np.array([[2.47, -2.47], [2.5, -2.47], [2.5, 2.47], [2.47, 2.47]]),

            # moving obstacle
            # np.array([[-2.4, 2.3], [-2.3, 2.3], [-2.3, 2.4], [-2.4, 2.4]]), # from back_old
            # np.array([[0.5, 0.0], [0.6, 0.0], [0.6, 0.1], [0.5, 0.1]]),  # from front_old

            # from far (A)
            # Spot A in fig 7
            # np.array([[-2.0, 2.0], [-1.9, 2.0], [-1.9, 2.1], [-2.0, 2.1]]),

            # for fuzzing
            np.array([[999.0, 2.0], [999.1, 2.0], [999.1, 2.1], [999.0, 2.1]]),

            # obs2 obs[-3] l_t_r
            np.array([[999.0, 2.0], [999.1, 2.0], [999.1, 2.1], [999.0, 2.1]]),
            # np.array([[5.6, 1.4], [5.7, 1.4], [5.7, 1.3], [5.6, 1.3]]),
            # np.array([[0.6, 1.4], [0.7, 1.4], [0.7, 1.3], [0.6, 1.3]]), #old
            # obs1 obs[-2] dia
            np.array([[-2.6, 1.4], [-2.5, 1.4], [-2.5, 1.5], [-2.6, 1.5]]),
        ]

        self.robots: list[Robot] = [] # 机器人列表
        for i in range(self.params.num_robots):
            self.robots.append(Robot(i+1, create_time=self.attack_time))
        self.robot1 = self.robots[0] # 领头机
        self.robot1.leader=True

        self.attack_robot = Robot(' attack', create_time=self.attack_time) # 攻击机
        self.attack_robot.sp = np.array([0.0, 2.0])

        self.metrics = Metrics()

        self.attack_strategy = attack_strategy

        self.time = 0
        self.attack_point: list[list] = list() # 攻击点列表
        self.is_show_attack_point = True # 是否显示攻击点
        self.is_rebuild_attack_path = True # 是否重建攻击路径

        self.rob_list = list()
        self.is_show_rob_pic = True # 是否展示健壮性变化趋势

    def start(self, P_path=None, traj_global_path=None) -> tuple:
        # 绘制地图
        plt.figure(num=1)
        draw_map(self.obstacles)
        plt.plot(self.xy_start[0], self.xy_start[1], 'bo', color='red', markersize=1, label='start')
        plt.plot(self.xy_goal[0], self.xy_goal[1], 'bo', color='green', markersize=1, label='goal')

        if P_path is None:
            P_long = rrt_path(self.obstacles, self.xy_start, self.xy_goal, self.params) # 获取长路径
            print('Path Shortenning...')
            P = ShortenPath(P_long, self.obstacles, smoothiters=50) # 缩短路径
        else:
            P = P_path

        if traj_global_path is None:
            traj_global = waypts2setpts(P, self.params) # 获取路径点
        else:
            traj_global = traj_global_path

        P = np.vstack([P, self.xy_start])

        # 绘制路线
        plt.plot(P[:,0], P[:,1], linewidth=1, color='orange', label='Global planner path')
        plt.pause(0.5)

        sp_ind = 0 # 用于指定位置下标

        # 指定无人机初始位置
        self.robot1.route = np.array([traj_global[0,:]])
        self.robot1.sp = self.robot1.route[-1,:]
        followers_sp = formation(self.params.num_robots, leader_des=self.robot1.sp, v=np.array([0,-1]), l=self.params.interrobots_dist)
        for i in range(len(followers_sp)):
            self.robots[i+1].sp = followers_sp[i]
            self.robots[i+1].route = np.array([followers_sp[i]])
        print('Start movement...')

        t0 = time.time()

        self.time = sp_ind

        plt.figure(num=2)

        # 开始执行无人机任务
        while True:
            plt.figure(1)
            self.t_array.append(time.time() - t0)

            # 中止性检查
            if self.abort_detection(sp_ind):
                break

            # 产生移动障碍物
            # if len(self.obstacles) > 2:
            #     self.move_obstacles()

            # 发动攻击
            if self.attack_strategy == 1:
                self.attack_strategy_random(traj_global, sp_ind, followers_sp)
            elif self.attack_strategy == 2:
                self.attack_strategy_SA(traj_global, sp_ind, followers_sp)
            elif self.attack_strategy == 3:
                self.attack_strategy_MA(traj_global, sp_ind, followers_sp)
            else:
                return

            centroid = 0
            for robot in self.robots: centroid += robot.sp / len(self.robots)
            self.metrics.centroid_path = np.vstack([self.metrics.centroid_path, centroid])
            # dists to robots from the centroid:
            dists = []
            for robot in self.robots:
                dists.append( norm(centroid-robot.sp) )
            # Formation size estimation
            self.metrics.mean_dists_array.append(np.mean(dists)) # Formation mean Radius
            self.metrics.max_dists_array.append(np.max(dists)) # Formation max Radius

            # Algorithm performance (CPU and memory usage)
            self.metrics.cpu_usage_array.append( cpu_usage() )
            self.metrics.memory_usage_array.append( memory_usage() )

            if self.params.visualize:
                plt.cla()
                self.visualize2D(traj_global, sp_ind, P)

                plt.draw()
                plt.pause(0.01)

            # update loop variable
            if sp_ind < traj_global.shape[0]-1 and norm(self.robot1.sp_global - centroid) < self.params.max_sp_dist: sp_ind += 1
            self.time += 1
        
        # 是否进行后处理，显示详细数据
        if self.params.postprocessing:
            t_array = t_array[1:]
            self.metrics.t_array = t_array
            self.metrics.centroid_path = self.metrics.centroid_path[1:,:]
            self.metrics.centroid_path_length = path_length(self.metrics.centroid_path)
            for robot in self.robots:
                self.metrics.robots.append(robot)

            postprocessing(self.metrics, self.params, visualize=1)
            if self.params.savedata:
                save_data(self.metrics)
        
        # 是否显示攻击点
        if self.is_show_attack_point:
            ap = np.array(self.attack_point)
            plt.figure(figsize=(10, 10))
            plt.title("Attack Point")
            plt.plot(ap[:, 0], ap[:, 1], '*', color='black', markersize=3)
            for i, ap in enumerate(self.attack_point):
                plt.annotate(str(i), xy=(self.attack_point[i][0], self.attack_point[i][1]), xytext=(self.attack_point[i][0], self.attack_point[i][1]))
            plt.savefig(self.attack_point_pic_to_save_file_path)

        # 是否重建攻击路径
        if self.is_rebuild_attack_path:
            self.build_attack_path()

        if self.is_show_rob_pic:
            self.draw_rob_change_pic()

        # close windows if Enter-button is pressed
        plt.draw()
        plt.pause(0.1)
        # input('Hit Enter to close')
        plt.close('all')

        if P_path is None and traj_global_path is None:
            return P.tolist(), traj_global.tolist()
        else:
            return None, None

    def abort_detection(self, sp_ind) -> bool:
        """中止性检查

        Returns:
            bool: 需要中止返回True，否则返回False
        """
        
        # 检测障碍物碰撞
        if (not sp_ind == 0) and (self.crash_obs_check()):
            self.recode_res("攻击成功，无人机碰撞到墙壁")
            return True
        
        # 检测无人机之间碰撞
        if (not sp_ind == 0) and (self.crash_uav_check()):
            self.recode_res("攻击成功，无人机之间发生碰撞")
            return True

        # 检测是否到达
        dist_to_goal = norm(self.robot1.sp - self.xy_goal)
        if dist_to_goal < self.params.goal_tolerance:
            self.recode_res("攻击失败，无人机到达目标")
            return True
        
        # 检测任务是否延迟
        if self.time > 300:
            self.recode_res("攻击成功，无人机没有到达目的地")
            return True

        return False
    
    def crash_obs_check(self) -> bool:
        """检测无人机是否碰撞到障碍物

        Returns:
            bool: _description_
        """
        for rbt in self.robots:
            dis, _ = nearest_obs(rbt.sp[0], rbt.sp[1], self.obstacles)
            if dis == -math.inf:
                rbt.monitor.logger.debug("无人机与障碍物发生碰撞！")
                return True
        return False
    
    def crash_uav_check(self) -> bool:
        """检测无人机之间是否发生碰撞

        Returns:
            bool: _description_
        """
        for i in range(len(self.robots)):
            for j in range(i+1, len(self.robots)):
                if norm(self.robots[i].sp - self.robots[j].sp) < 0.03:
                    return True
        return False

    @staticmethod
    def _generate_point_in_intersection(c1, r1, c2, r2):
        """
        Generates a random point within the intersection of two circles using rejection sampling.
        c1, r1: center and radius of the first circle.
        c2, r2: center and radius of the second circle.
        """
        # Bounding box for the intersection
        x_min = max(c1[0] - r1, c2[0] - r2)
        x_max = min(c1[0] + r1, c2[0] + r2)
        y_min = max(c1[1] - r1, c2[1] - r2)
        y_max = min(c1[1] + r1, c2[1] + r2)

        # Safety check in case there is no intersection
        if x_min > x_max or y_min > y_max:
            dist = np.linalg.norm(c2 - c1)
            direction = (c2 - c1) / dist if dist > 0 else np.array([1, 0])
            return c1 + direction * r1 * 0.9

        max_attempts = 100 # Safety break
        for _ in range(max_attempts):
            point = np.array([random.uniform(x_min, x_max), random.uniform(y_min, y_max)])
            if np.linalg.norm(point - c1) <= r1 and np.linalg.norm(point - c2) <= r2:
                return point
        
        dist = np.linalg.norm(c2 - c1)
        direction = (c2 - c1) / dist if dist > 0 else np.array([1, 0])
        return c1 + direction * r1 * 0.9

    def attack_strategy_random(self, traj_global, sp_ind, followers_sp):
        """
        随机攻击策略：通过随机生成攻击点进行多次试探的攻击策略。
        1. 保存当前状态并计算基线健壮性。
        2. 进行20次随机攻击试探，每次随机生成攻击方向和距离。
        3. 模拟每次攻击并评估集群健壮性。
        4. 选择导致集群健壮性最低的攻击点作为最终攻击目标。
        """
        # Phase 1: Preparation and Snapshot
        store_data = list()
        for rob in self.robots:
            store_data.append((rob.sp, rob.route))
        store_data.append((self.attack_robot.sp, self.attack_robot.route))

        # Phase 2: Calculate Baseline Robustness
        self.attack_robot.monitor.logger.debug("开始计算攻击机不移动干扰下的集群健壮性")
        attack_rob_obs = poses2polygons([self.attack_robot.sp])
        self.robot1.sp_global = traj_global[sp_ind,:]
        self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)
        followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
        for i in range(len(followers_sp_global)):
            self.robots[i+1].sp_global = followers_sp_global[i]
        for p in range(len(followers_sp)): 
            robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if i!=p]
            robots_obstacles = poses2polygons(robots_obstacles_sp)
            obstacles1 = np.array(self.obstacles + robots_obstacles)
            self.robots[p+1].local_planner(obstacles1, self.params)
        
        normal_rob = 0.0
        for rob in self.robots:
            normal_rob += rob.monitor.rule1_monitor(self.get_info(rob))
        self.attack_robot.monitor.logger.debug("计算结束，无人机集群在攻击机不移动情况下的健壮性为%s" % (normal_rob))
        
        # 发动攻击
        min_rob = normal_rob
        min_rob_global_sp = np.array([0, 0])
        self.attack_robot.monitor.logger.debug("攻击机开始进行攻击试探")

        for i in range(20):
            # 进行无人机位置信息还原
            for x in range(len(self.robots)):
                self.robots[x].sp = store_data[x][0]
                self.robots[x].route = store_data[x][1]

            self.attack_robot.sp = store_data[-1][0]
            self.attack_robot.route = store_data[-1][1]
            self.robot1.sp_global = traj_global[sp_ind,:]

            # 随机产生一个方向
            theta = random.random() * 2 * math.pi
            r = random.uniform(0, 5)
            x, y = math.sin(theta) * (r ** 0.5), math.cos(theta) * (r ** 0.5)
            self.attack_robot.sp_global = np.array([x, y])
            self.attack_robot.monitor.logger.debug("随机产生攻击方向%s: %s" % (i, self.attack_robot.sp_global))

            # 攻击机开始移动
            self.attack_robot.local_planner(self.obstacles, self.params)
            
            # 检测攻击机和无人机之间是否发生碰撞，如果发生，则不考虑该位置
            if self.uav_crash_check():
                continue
            
            # 其他无人机开始移动
            attack_rob_obs = poses2polygons([self.attack_robot.sp])
            self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j+1].sp_global = followers_sp_global[j]
            for p in range(len(followers_sp)): # formation poses correction with local planner
                # robots repel from each other inside the formation
                robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if k!=p] # all poses except the robot[p]
                robots_obstacles = poses2polygons(robots_obstacles_sp) # each drone is defined as a small cube for inter-robots collision avoidance
                obstacles1 = np.array(self.obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
                # follower robot's position correction with local planner
                self.robots[p+1].local_planner(obstacles1, self.params)
                # followers_sp[p] = robots[p+1].sp
            
            # 计算总体健壮性
            total_rob = 0.0
            for rob in self.robots:
                total_rob += rob.monitor.rule1_monitor(self.get_info(rob))
            self.attack_robot.monitor.logger.debug("随机攻击方向%s导致最终无人机蜂群健壮性为: %s" % (i, total_rob))
            self.rob_list.append(min_rob)
            if total_rob < min_rob:
                min_rob = total_rob
                min_rob_global_sp = self.attack_robot.sp_global
        
        # 无人机违反策略
        if min_rob == -math.inf:
            self.is_crash = True
        
        # 进行无人机位置信息还原
        for x in range(len(self.robots)):
            self.robots[x].sp = store_data[x][0]
            self.robots[x].route = store_data[x][1]

        self.attack_robot.sp = store_data[-1][0]
        self.attack_robot.route = store_data[-1][1]
        self.robot1.sp_global = traj_global[sp_ind,:]
        
        if min_rob >= normal_rob:
            self.attack_robot.monitor.logger.debug("随机产生的攻击未对集群造成负面影响，攻击机直接飞向领头机")
            self.attack_robot.sp_global = self.robot1.sp
            # attack_robot.local_planner(obs_total, params)

            # attack_direction_vector = attack_robot.sp_global - attack_robot.sp
            # attack_robot.sp = attack_robot.sp + 0.01 *  params.drone_vel * (attack_direction_vector / norm(attack_direction_vector))
            # attack_robot.route = np.vstack( [attack_robot.route, attack_robot.sp] )

            self.attack_robot.local_planner(self.obstacles, self.params)

            attack_rob_obs = poses2polygons([self.attack_robot.sp])
            self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j+1].sp_global = followers_sp_global[j]
            for p in range(len(followers_sp)): 
                robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if k!=p]
                robots_obstacles = poses2polygons( robots_obstacles_sp )
                obstacles1 = np.array(self.obstacles + robots_obstacles)
                self.robots[p+1].local_planner(obstacles1, self.params)
                followers_sp[p] = self.robots[p+1].sp
            return
        
        self.attack_robot.monitor.logger.debug("攻击机攻击试探结束，最终确定攻击方向为: %s，将导致无人机集群健壮性变成%s" % (min_rob_global_sp, min_rob))
        self.attack_robot.sp_global = min_rob_global_sp
        # attack_robot.local_planner(obs_total, params)

        # attack_direction_vector = attack_robot.sp_global - attack_robot.sp
        # attack_robot.sp = attack_robot.sp + 0.01 * params.drone_vel * (attack_direction_vector / norm(attack_direction_vector))
        # attack_robot.route = np.vstack( [attack_robot.route, attack_robot.sp] )

        self.attack_robot.local_planner(self.obstacles, self.params)

        attack_rob_obs = poses2polygons([self.attack_robot.sp])
        self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)

        followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
        for i in range(len(followers_sp_global)):
            self.robots[i+1].sp_global = followers_sp_global[i]
        for p in range(len(followers_sp)): # formation poses correction with local planner
            # robots repel from each other inside the formation
            robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if i!=p] # all poses except the robot[p]
            robots_obstacles = poses2polygons(robots_obstacles_sp) # each drone is defined as a small cube for inter-robots collision avoidance
            obstacles1 = np.array(self.obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
            # follower robot's position correction with local planner
            self.robots[p+1].local_planner(obstacles1, self.params)
            followers_sp[p] = self.robots[p+1].sp

    def attack_strategy_SA(self, traj_global, sp_ind, followers_sp):
        """
        SA攻击策略：结合Katz中心性和多次试探的精确打击策略。
        1. 识别并构建与攻击机可交互的无人机子图。
        2. 使用Katz中心性找到关键"受害者"无人机。
        3. 在20次循环中，每次都在攻击机可达范围和受害者感知范围的交集中生成一个随机攻击点。
        4. 模拟并评估每个攻击点，选择最优方案。
        """
        # Phase 1: Preparation and Snapshot
        store_data = list()
        for rob in self.robots:
            store_data.append((rob.sp, rob.route))
        store_data.append((self.attack_robot.sp, self.attack_robot.route))

        # Phase 2: Calculate Baseline Robustness
        self.attack_robot.monitor.logger.debug("开始计算攻击机不移动干扰下的集群健壮性")
        attack_rob_obs = poses2polygons([self.attack_robot.sp])
        self.robot1.sp_global = traj_global[sp_ind,:]
        self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)
        followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
        for i in range(len(followers_sp_global)):
            self.robots[i+1].sp_global = followers_sp_global[i]
        for p in range(len(followers_sp)):
            robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if i!=p]
            robots_obstacles = poses2polygons(robots_obstacles_sp)
            obstacles1 = np.array(self.obstacles + robots_obstacles)
            self.robots[p+1].local_planner(obstacles1, self.params)

        normal_rob = 0.0
        for rob in self.robots:
            normal_rob += rob.monitor.rule1_monitor(self.get_info(rob))
        self.attack_robot.monitor.logger.debug("计算结束，无人机集群在攻击机不移动情况下的健壮性为%s" % (normal_rob))

        # 发动攻击
        min_rob = normal_rob
        min_rob_global_sp = np.array([0, 0])
        self.attack_robot.monitor.logger.debug("攻击机开始进行攻击试探")

        # --- NEW STRATEGY 2 CORE LOGIC ---
        # 1. 识别潜在受害者并找到关键节点
        t = 0.1  # 时间窗口
        attacker_reachable_radius = self.params.drone_vel * t
        potential_victims = []
        for rob in self.robots:
            dist = np.linalg.norm(self.attack_robot.sp - rob.sp)
            if dist < attacker_reachable_radius + self.params.influence_radius:
                potential_victims.append(rob)

        victim_robot = None
        if potential_victims:
            try:
                G = nx.Graph()
                for i in range(len(potential_victims)):
                    for j in range(i + 1, len(potential_victims)):
                        d = np.linalg.norm(potential_victims[i].sp - potential_victims[j].sp)
                        if d < self.params.interrobots_dist * 1.5:
                            G.add_edge(potential_victims[i].id, potential_victims[j].id)
                
                if G.number_of_nodes() > 0:
                    centrality = nx.katz_centrality_numpy(G)
                    victim_id = max(centrality, key=centrality.get)
                    for rob in potential_victims:
                        if rob.id == victim_id:
                            victim_robot = rob
                            break
                    if victim_robot:
                        self.attack_robot.monitor.logger.info(f"Katz中心性分析完成，锁定关键无人机: {victim_robot.id}")
                else:
                    victim_robot = random.choice(potential_victims)
                    self.attack_robot.monitor.logger.warning("无法构建有效拓扑，随机选择一个可达目标")
            except Exception as e:
                self.attack_robot.monitor.logger.error(f"Katz中心性分析失败: {e}, 随机选择一个可达目标")
                victim_robot = random.choice(potential_victims)

        if victim_robot is None:
            victim_robot = self.robot1 # 兜底策略
            self.attack_robot.monitor.logger.warning("无可达目标，选择领头机作为目标")
        # --- END OF CORE LOGIC ---

        for i in range(20):
            # 进行无人机位置信息还原
            for x in range(len(self.robots)):
                self.robots[x].sp = store_data[x][0]
                self.robots[x].route = store_data[x][1]
            self.attack_robot.sp = store_data[-1][0]
            self.attack_robot.route = store_data[-1][1]
            self.robot1.sp_global = traj_global[sp_ind,:]

            # 3. 在交集内生成候选攻击点
            candidate_point = self._generate_point_in_intersection(
                self.attack_robot.sp,
                attacker_reachable_radius,
                victim_robot.sp,
                self.params.influence_radius
            )
            self.attack_robot.sp_global = candidate_point
            self.attack_robot.monitor.logger.debug("在受害者 %s 周围产生攻击点 %s: %s" % (victim_robot.id, i, self.attack_robot.sp_global))

            # 攻击机开始移动
            self.attack_robot.local_planner(self.obstacles, self.params)
            
            # 检测攻击机和无人机之间是否发生碰撞
            if self.uav_crash_check():
                continue
            
            # 其他无人机开始移动
            attack_rob_obs = poses2polygons([self.attack_robot.sp])
            self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j+1].sp_global = followers_sp_global[j]
            for p in range(len(followers_sp)):
                robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if k!=p]
                robots_obstacles = poses2polygons(robots_obstacles_sp)
                obstacles1 = np.array(self.obstacles + robots_obstacles)
                self.robots[p+1].local_planner(obstacles1, self.params)
            
            # 计算总体健壮性
            total_rob = 0.0
            for rob in self.robots:
                total_rob += rob.monitor.rule1_monitor(self.get_info(rob))
            self.attack_robot.monitor.logger.debug("随机攻击方向%s导致最终无人机蜂群健壮性为: %s" % (i, total_rob))
            self.rob_list.append(min_rob)
            if total_rob < min_rob:
                min_rob = total_rob
                min_rob_global_sp = self.attack_robot.sp_global
        
        # 无人机违反策略
        if min_rob == -math.inf:
            self.is_crash = True
        
        # 进行无人机位置信息还原
        for x in range(len(self.robots)):
            self.robots[x].sp = store_data[x][0]
            self.robots[x].route = store_data[x][1]
        self.attack_robot.sp = store_data[-1][0]
        self.attack_robot.route = store_data[-1][1]
        self.robot1.sp_global = traj_global[sp_ind,:]
        
        if min_rob >= normal_rob:
            self.attack_robot.monitor.logger.debug("随机产生的攻击未对集群造成负面影响，攻击机直接飞向领头机")
            self.attack_robot.sp_global = self.robot1.sp
            self.attack_robot.local_planner(self.obstacles, self.params)
            attack_rob_obs = poses2polygons([self.attack_robot.sp])
            self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)
            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j+1].sp_global = followers_sp_global[j]
            for p in range(len(followers_sp)):
                robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if k!=p]
                robots_obstacles = poses2polygons( robots_obstacles_sp )
                obstacles1 = np.array(self.obstacles + robots_obstacles)
                self.robots[p+1].local_planner(obstacles1, self.params)
                followers_sp[p] = self.robots[p+1].sp
            return
        
        self.attack_robot.monitor.logger.debug("攻击机攻击试探结束，最终确定攻击方向为: %s，将导致无人机集群健壮性变成%s" % (min_rob_global_sp, min_rob))
        self.attack_robot.sp_global = min_rob_global_sp
        self.attack_robot.local_planner(self.obstacles, self.params)
        attack_rob_obs = poses2polygons([self.attack_robot.sp])
        self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)
        followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
        for i in range(len(followers_sp_global)):
            self.robots[i+1].sp_global = followers_sp_global[i]
        for p in range(len(followers_sp)):
            robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if i!=p]
            robots_obstacles = poses2polygons(robots_obstacles_sp)
            obstacles1 = np.array(self.obstacles + robots_obstacles)
            self.robots[p+1].local_planner(obstacles1, self.params)
            followers_sp[p] = self.robots[p+1].sp

    def attack_strategy_MA(self, traj_global, sp_ind, followers_sp):
        """
        MA攻击策略：每轮寻找全局最关键无人机的精确打击策略。
        1. 每轮都构建全局无人机网络拓扑。
        2. 使用Katz中心性找到全局最关键的无人机。
        3. 在20次循环中，每次都在关键无人机感知范围内随机生成攻击点。
        4. 攻击机可瞬移到任意位置（速度无限快）。
        """
        # Phase 1: Preparation and Snapshot
        store_data = list()
        for rob in self.robots:
            store_data.append((rob.sp, rob.route))
        store_data.append((self.attack_robot.sp, self.attack_robot.route))

        # Phase 2: Calculate Baseline Robustness
        self.attack_robot.monitor.logger.debug("开始计算攻击机不移动干扰下的集群健壮性")
        attack_rob_obs = poses2polygons([self.attack_robot.sp])
        self.robot1.sp_global = traj_global[sp_ind,:]
        self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)
        followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
        for i in range(len(followers_sp_global)):
            self.robots[i+1].sp_global = followers_sp_global[i]
        for p in range(len(followers_sp)):
            robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if i!=p]
            robots_obstacles = poses2polygons(robots_obstacles_sp)
            obstacles1 = np.array(self.obstacles + robots_obstacles)
            self.robots[p+1].local_planner(obstacles1, self.params)

        normal_rob = 0.0
        for rob in self.robots:
            normal_rob += rob.monitor.rule1_monitor(self.get_info(rob))
        self.attack_robot.monitor.logger.debug("计算结束，无人机集群在攻击机不移动情况下的健壮性为%s" % (normal_rob))

        # 发动攻击
        min_rob = normal_rob
        min_rob_global_sp = np.array([0, 0])
        self.attack_robot.monitor.logger.debug("攻击机开始进行MA攻击试探")

        for i in range(20):
            # 进行无人机位置信息还原
            for x in range(len(self.robots)):
                self.robots[x].sp = store_data[x][0]
                self.robots[x].route = store_data[x][1]
            self.attack_robot.sp = store_data[-1][0]
            self.attack_robot.route = store_data[-1][1]
            self.robot1.sp_global = traj_global[sp_ind,:]

            # --- MA STRATEGY CORE LOGIC ---
            # 每轮都寻找全局最关键无人机
            try:
                import networkx as nx
                G = nx.Graph()
                # 构建全局网络拓扑
                for j in range(len(self.robots)):
                    for k in range(j + 1, len(self.robots)):
                        d = np.linalg.norm(self.robots[j].sp - self.robots[k].sp)
                        if d < self.params.interrobots_dist * 1.5:
                            G.add_edge(self.robots[j].id, self.robots[k].id)
                
                victim_robot = None
                if G.number_of_nodes() > 0:
                    centrality = nx.katz_centrality_numpy(G)
                    victim_id = max(centrality, key=centrality.get)
                    for rob in self.robots:
                        if rob.id == victim_id:
                            victim_robot = rob
                            break
                    if victim_robot:
                        self.attack_robot.monitor.logger.info(f"第{i}轮：Katz中心性分析完成，锁定全局关键无人机: {victim_robot.id}")
                else:
                    victim_robot = self.robot1  # 兜底策略
                    self.attack_robot.monitor.logger.warning(f"第{i}轮：无法构建有效拓扑，选择领头机作为目标")
            except Exception as e:
                self.attack_robot.monitor.logger.error(f"第{i}轮：Katz中心性分析失败: {e}, 选择领头机作为目标")
                victim_robot = self.robot1

            if victim_robot is None:
                victim_robot = self.robot1

            # 在关键无人机感知范围内随机生成攻击点（攻击机可瞬移）
            theta = random.random() * 2 * math.pi
            r = random.uniform(0, self.params.influence_radius)
            x = victim_robot.sp[0] + r * math.cos(theta)
            y = victim_robot.sp[1] + r * math.sin(theta)
            candidate_point = np.array([x, y])
            
            # 攻击机瞬移到目标位置
            self.attack_robot.sp_global = candidate_point
            self.attack_robot.sp = candidate_point  # 瞬移，直接设置位置
            self.attack_robot.monitor.logger.debug("第%d轮：攻击机瞬移到关键无人机 %s 感知范围内的攻击点: %s" % (i, victim_robot.id, self.attack_robot.sp_global))
            # --- END OF MA CORE LOGIC ---
            
            # 检测攻击机和无人机之间是否发生碰撞
            if self.uav_crash_check():
                continue
            
            # 其他无人机开始移动
            attack_rob_obs = poses2polygons([self.attack_robot.sp])
            self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j+1].sp_global = followers_sp_global[j]
            for p in range(len(followers_sp)):
                robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if k!=p]
                robots_obstacles = poses2polygons(robots_obstacles_sp)
                obstacles1 = np.array(self.obstacles + robots_obstacles)
                self.robots[p+1].local_planner(obstacles1, self.params)
            
            # 计算总体健壮性
            total_rob = 0.0
            for rob in self.robots:
                total_rob += rob.monitor.rule1_monitor(self.get_info(rob))
            self.attack_robot.monitor.logger.debug("第%d轮MA攻击导致最终无人机蜂群健壮性为: %s" % (i, total_rob))
            self.rob_list.append(total_rob)
            if total_rob < min_rob:
                min_rob = total_rob
                min_rob_global_sp = self.attack_robot.sp_global
        
        # 无人机违反策略
        if min_rob == -math.inf:
            self.is_crash = True
        
        # 进行无人机位置信息还原
        for x in range(len(self.robots)):
            self.robots[x].sp = store_data[x][0]
            self.robots[x].route = store_data[x][1]
        self.attack_robot.sp = store_data[-1][0]
        self.attack_robot.route = store_data[-1][1]
        self.robot1.sp_global = traj_global[sp_ind,:]
        
        if min_rob >= normal_rob:
            self.attack_robot.monitor.logger.debug("MA攻击未对集群造成负面影响，攻击机直接飞向领头机")
            self.attack_robot.sp_global = self.robot1.sp
            self.attack_robot.sp = self.robot1.sp  # 瞬移到领头机位置
            attack_rob_obs = poses2polygons([self.attack_robot.sp])
            self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)
            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j+1].sp_global = followers_sp_global[j]
            for p in range(len(followers_sp)):
                robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if k!=p]
                robots_obstacles = poses2polygons( robots_obstacles_sp )
                obstacles1 = np.array(self.obstacles + robots_obstacles)
                self.robots[p+1].local_planner(obstacles1, self.params)
                followers_sp[p] = self.robots[p+1].sp
            return
        
        self.attack_robot.monitor.logger.debug("MA攻击试探结束，最终确定攻击方向为: %s，将导致无人机集群健壮性变成%s" % (min_rob_global_sp, min_rob))
        self.attack_robot.sp_global = min_rob_global_sp
        self.attack_robot.sp = min_rob_global_sp  # 瞬移到最优攻击位置
        attack_rob_obs = poses2polygons([self.attack_robot.sp])
        self.robot1.local_planner(np.array(self.obstacles + attack_rob_obs), self.params)
        followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
        for i in range(len(followers_sp_global)):
            self.robots[i+1].sp_global = followers_sp_global[i]
        for p in range(len(followers_sp)):
            robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [self.robot1.sp, self.attack_robot.sp]) if i!=p]
            robots_obstacles = poses2polygons(robots_obstacles_sp)
            obstacles1 = np.array(self.obstacles + robots_obstacles)
            self.robots[p+1].local_planner(obstacles1, self.params)
            followers_sp[p] = self.robots[p+1].sp



    def recompute_attack_robot_sp(self, min_rob_pos):
        pass
    
    def uav_crash_check(self) -> bool:
        """判断攻击机和无人机之间是否发生碰撞

        Returns:
            bool: 检测结果
        """
        for rbt in self.robots:
            # D = math.sqrt(math.pow((self.attack_robot.sp[0] - rbt.sp[0]), 2) + math.pow((self.attack_robot.sp[1] - rbt.sp[1]), 2))
            D = norm(self.attack_robot.sp - rbt.sp)
            if D < 0.07:
                return True
        return False
    
    def cal_perceptual_robustness(self, rbt: Robot, traj_global, sp_ind, radius):
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
            rbt.local_planner(np.array(self.obstacles + point_obs), self.params)

            rob = rbt.monitor.rule1_monitor(self.get_info(rbt))
            rbt.monitor.logger.debug("周围点%s的健壮性为%s" % (rbt.perceptual_robustness_list[i], rob))
            rbt.perceptual_robustness_list[i].append(rob)

            # 无人机数据还原
            rbt.sp = rbt_sp
            rbt.route = rbt_route

    def cal_perceptual_robustness_2(self, rbt: Robot, traj_global, sp_ind, radius, followers_sp):

        # 保存位置信息
        rbt_sp = rbt.sp
        rbt_route = rbt.route
        store_data = list()
        for rob in self.robots:
            store_data.append([rob.sp, rob.route])
        
        rbt.perceptual_robustness_list.clear()

        self.robot1.sp_global = traj_global[sp_ind,:]

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
            if self.check_outermost_layer(rbt.perceptual_robustness_list[i]):
                point_obs = poses2polygons([rbt.perceptual_robustness_list[i]])
                self.robot1.local_planner(np.array(self.obstacles + point_obs), self.params)

                followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
                for j in range(len(followers_sp_global)):
                    self.robots[j+1].sp_global = followers_sp_global[j]
                for p in range(len(followers_sp)): # formation poses correction with local planner
                    # robots repel from each other inside the formation
                    robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [self.robot1.sp, rbt.perceptual_robustness_list[i]]) if k!=p] # all poses except the robot[p]
                    robots_obstacles = poses2polygons(robots_obstacles_sp) # each drone is defined as a small cube for inter-robots collision avoidance
                    obstacles1 = np.array(self.obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
                    # follower robot's position correction with local planner
                    self.robots[p+1].local_planner(obstacles1, self.params)
                    # followers_sp[p] = robots[p+1].sp

                total_rob = 0.0
                for rob in self.robots:
                    total_rob += rob.monitor.rule1_monitor(self.get_info(rob))
                rbt.monitor.logger.debug("攻击机攻击方向%s导致无人机蜂群健壮性为：%s" % (str(rbt.perceptual_robustness_list[i]), total_rob))
                rbt.perceptual_robustness_list[i].append(total_rob)
            else:
                rbt.perceptual_robustness_list[i].append(math.inf)
            
            # 无人机数据还原
            rbt.sp = rbt_sp
            rbt.route = rbt_route
            for x in range(len(self.robots)):
                self.robots[x].sp = store_data[x][0]
                self.robots[x].route = store_data[x][1]

    def cal_perceptual_robustness_3(self, rbt: Robot, radius, followers_sp):

        # 计算rbt周围扇形的中心点
        rbt.perceptual_robustness_list.clear()
        rbt.min_perceptual_robustness = math.inf

        r = radius
        count = rbt.curve_count # 没用了
        count2 = rbt.straight_count
        x_c, y_c = rbt.sp[0], rbt.sp[1]

        # sub_r = r / count * ((2 * (count - 1) + 1) / 2)
        sub_r = r - 1 / 2 * 0.01 * self.params.drone_vel
        for j in range(count2):
            x = np.cos(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + x_c
            y = np.sin(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + y_c
            if x >= -2.5 and x <=2.5 and y >= -2.5 and y <= 2.5:
                rbt.perceptual_robustness_list.append([x, y])
        
        rbt.monitor.logger.debug("开始计算周围感知健壮性")
        for i in range(len(rbt.perceptual_robustness_list)):
            # 检测该点位于所有无人机的最外层才计算健壮性
            if self.check_outermost_layer(rbt.perceptual_robustness_list[i]):
                # 直接复制所有的无人机信息
                robots_copy: list[Robot] = [None] * self.params.num_robots
                for j in range(len(self.robots)):
                    robots_copy[j] = self.copy_robot(self.robots[j])

                point_obs = poses2polygons([rbt.perceptual_robustness_list[i]])
                robots_copy[0].local_planner(np.array(self.obstacles + point_obs), self.params)

                followers_sp_global = formation(self.params.num_robots, robots_copy[0].sp_global, v=normalize(robots_copy[0].sp_global - robots_copy[0].sp), l=self.params.interrobots_dist)
                for j in range(len(followers_sp_global)):
                    robots_copy[j+1].sp_global = followers_sp_global[j]
                for p in range(len(followers_sp)):
                    robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [robots_copy[0].sp, rbt.perceptual_robustness_list[i]]) if k!=p]
                    robots_obstacles = poses2polygons(robots_obstacles_sp)
                    obstacles1 = np.array(self.obstacles + robots_obstacles)
                    robots_copy[p+1].local_planner(obstacles1, self.params)
                
                # ------------------规则一--------------------------
                total_rob = 0.0
                for rob in robots_copy:
                    total_rob += rob.monitor.rule1_monitor(self.get_info(rob))

                # ------------------规则二--------------------------
                # total_rob += rbt.monitor.multi_rule2_monitor(self.get_rule2_info(robots_copy))

                rbt.monitor.logger.debug("攻击机攻击方向%s导致无人机蜂群健壮性为：%s" % (str(rbt.perceptual_robustness_list[i]), total_rob))

                if total_rob < rbt.min_perceptual_robustness:
                    rbt.min_perceptual_robustness = total_rob
                    rbt.min_perceptual_robustness_pos = rbt.perceptual_robustness_list[i]
    
    def cal_attack_perceptual_robustness(self, rbt: Robot, traj_global, sp_ind, radius, followers_sp):

        # 保存位置信息
        rbt_sp = rbt.sp
        rbt_route = rbt.route
        store_data = list()
        for rob in self.robots:
            store_data.append([rob.sp, rob.route])

        rbt.perceptual_robustness_list.clear()

        self.robot1.sp_global = traj_global[sp_ind,:]

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
            rbt.local_planner(self.obstacles, self.params)

            if self.uav_crash_check():
                rbt.perceptual_robustness_list[i].append(math.inf)
                rbt.sp = rbt_sp
                rbt.route = rbt_route
                continue

            rbt_obs = poses2polygons([rbt.sp])
            self.robot1.local_planner(np.array(self.obstacles + rbt_obs), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j+1].sp_global = followers_sp_global[j]
            for p in range(len(followers_sp)): # formation poses correction with local planner
                # robots repel from each other inside the formation
                robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [self.robot1.sp, rbt.sp]) if k!=p] # all poses except the robot[p]
                robots_obstacles = poses2polygons(robots_obstacles_sp) # each drone is defined as a small cube for inter-robots collision avoidance
                obstacles1 = np.array(self.obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
                # follower robot's position correction with local planner
                self.robots[p+1].local_planner(obstacles1, self.params)
                # followers_sp[p] = robots[p+1].sp

            total_rob = 0.0
            for rob in self.robots:
                total_rob += rob.monitor.rule1_monitor(self.get_info(rob))
            rbt.monitor.logger.debug("攻击机攻击方向%s导致无人机蜂群健壮性为：%s" % (str(rbt.perceptual_robustness_list[i]), total_rob))
            rbt.perceptual_robustness_list[i].append(total_rob)

            # 无人机数据还原
            rbt.sp = rbt_sp
            rbt.route = rbt_route
            for x in range(len(self.robots)):
                self.robots[x].sp = store_data[x][0]
                self.robots[x].route = store_data[x][1]

    def cal_attack_perceptual_robustness_2(self, rbt: Robot, traj_global, sp_ind, radius, followers_sp):

        # 保存位置信息
        rbt_sp = rbt.sp
        rbt_route = rbt.route
        store_data = list()
        for rob in self.robots:
            store_data.append([rob.sp, rob.route])

        rbt.perceptual_robustness_list.clear()
        rbt.min_perceptual_robustness = math.inf

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
            rbt.local_planner(self.obstacles, self.params)

            if self.uav_crash_check():
                rbt.perceptual_robustness_list[i].append(math.inf)
                rbt.sp = rbt_sp
                rbt.route = rbt_route
                continue

            rbt_obs = poses2polygons([rbt.sp])
            self.robot1.local_planner(np.array(self.obstacles + rbt_obs), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global, v=normalize(self.robot1.sp_global - self.robot1.sp), l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j+1].sp_global = followers_sp_global[j]
            for p in range(len(followers_sp)): # formation poses correction with local planner
                # robots repel from each other inside the formation
                robots_obstacles_sp = [x for k,x in enumerate(followers_sp + [self.robot1.sp, rbt.sp]) if k!=p] # all poses except the robot[p]
                robots_obstacles = poses2polygons(robots_obstacles_sp) # each drone is defined as a small cube for inter-robots collision avoidance
                obstacles1 = np.array(self.obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
                # follower robot's position correction with local planner
                self.robots[p+1].local_planner(obstacles1, self.params)
                # followers_sp[p] = robots[p+1].sp

            # ------------------规则一--------------------------
            total_rob = 0.0
            for rob in self.robots:
                total_rob += rob.monitor.rule1_monitor(self.get_info(rob))

            # ------------------规则二--------------------------
            # total_rob += rbt.monitor.multi_rule2_monitor(self.get_rule2_info(self.robots))
            rbt.monitor.logger.debug("攻击机攻击方向%s导致无人机蜂群健壮性为：%s" % (str(rbt.perceptual_robustness_list[i]), total_rob))
            # rbt.perceptual_robustness_list[i].append(total_rob)

            if total_rob < rbt.min_perceptual_robustness:
                rbt.min_perceptual_robustness = total_rob
                rbt.min_perceptual_robustness_pos = rbt.perceptual_robustness_list[i]

            # 无人机数据还原
            rbt.sp = rbt_sp
            rbt.route = rbt_route
            for x in range(len(self.robots)):
                self.robots[x].sp = store_data[x][0]
                self.robots[x].route = store_data[x][1]
    
    def check_outermost_layer(self, point: list) -> bool:
        for rbt in self.robots:
            min_curve_dis = (rbt.curve_count - 1) / rbt.curve_count * self.params.influence_radius
            dis = norm(point - rbt.sp)
            if dis < min_curve_dis:
                return False
        return True
    
    def rob_map(self, rbt: Robot, radius, nrows=500, ncols=500):

        # 计算感知范围的最短距离
        min_curve_dis = (rbt.curve_count - 1) / rbt.curve_count * self.params.influence_radius


        x1 = meters2grid(rbt.sp[1] - radius)
        x2 = meters2grid(rbt.sp[1] + radius)
        y1 = meters2grid(rbt.sp[0] - radius)
        y2 = meters2grid(rbt.sp[0] + radius)
        
        # 计算矩阵上方
        if x1 > 0:
            # self.grid[0: ncols + 1, 0: grid_circular_matrix[0][1]] += rbt.no_influence_rob
            self.grid[0: x1, 0: ncols + 1] += rbt.no_influence_rob
        
        # 计算矩阵下方
        if x2 < 500:
            self.grid[x2 + 1: nrows + 1, 0: ncols + 1] += rbt.no_influence_rob
        
        # 计算矩阵左方
        if y1 > 0:
            self.grid[x1: x2 + 1, 0: y1] += rbt.no_influence_rob
        
        # 计算矩阵右方
        if y2 < 500:
            self.grid[x1: x2 + 1, y2 + 1: 500 + 1] += rbt.no_influence_rob
        
        for i in range(x1, x2 + 1):
            for j in range(y1, y2 + 1):
                # 计算位置点到无人机位置的距离
                dis = norm(grid2meters([j, i]) - rbt.sp)
                if dis > self.params.influence_radius:
                    # 位置点在无人机感知范围外
                    self.grid[i][j] += rbt.no_influence_rob
                elif dis >= min_curve_dis:
                    # 位置点在无人机感知圆环范围内
                    min_dis_pos = 0
                    min_dis = math.inf

                    # 计算位置点到各个感知点的距离，最短距离所在的位置点就是代表的位置点
                    for p in range(len(rbt.perceptual_robustness_list)):
                        perceptual_dis = norm(grid2meters([j, i]) - rbt.perceptual_robustness_list[p][0: 2])
                        if perceptual_dis < min_dis:
                            min_dis = perceptual_dis
                            min_dis_pos = p

                    self.grid[i][j] += rbt.perceptual_robustness_list[min_dis_pos][2]
                else:
                    # 位置点在无人机感知圆环范围外，攻击机下一步不可能到达该位置
                    self.grid[i][j] += math.inf
                
                # if self.grid[i][j] < min_rob:
                #     min_rob = self.grid[i][j]
                #     min_rob_pos = grid2meters([j, i])

        # return min_rob, min_rob_pos

    def visualize2D(self, traj_global, sp_ind, P):
        smiley = parse_path("""M458 2420 c-215 -38 -368 -257 -329 -469 34 -182 175 -314 354 -329 l57 -4 0 45 0 44 -42 7 c-101 16 -187 79 -236 171 -37 69 -38 187 -4 257 30 60 90 120 150 150 70 34 188 33 258 -4 89 -47 153 -136 169 -235 l7 -43 50 0 51 0 -6 59 c-13 147 -124 285 -268 334 -60 20 -152 28 -211 17z M1940 2417 c-172 -39 -302 -181 -317 -347 l-6 -60 51 0 50 0 12 52 c14 70 49 126 110 181 118 106 284 100 399 -14 64 -64 86 -120 86 -214 0 -67 -5 -88 -27 -130 -49 -92 -135 -155 -236 -171 l-42 -7 0 -49 0 -50 58 4 c115 8 242 91 306 200 36 61 59 177 51 248 -30 244 -260 410 -495 357z M506 2038 c-9 -12 -16 -41 -16 -64 0 -39 11 -56 158 -240 87 -110 161 -205 166 -212 5 -9 10 -382 6 -494 0 -3 -74 -97 -165 -208 l-165 -202 0 -52 c0 -68 18 -86 86 -86 40 0 55 5 80 28 17 15 112 89 211 166 l180 138 239 0 239 -1 209 -165 c203 -162 210 -166 256 -166 60 0 80 20 80 81 0 43 -8 55 -170 264 l-170 220 0 230 c0 202 2 233 18 257 9 15 86 108 170 208 l152 180 0 54 c0 65 -19 86 -76 86 -36 0 -58 -15 -234 -151 -107 -83 -205 -158 -217 -166 -19 -12 -67 -15 -260 -15 l-238 1 -209 165 -209 166 -53 0 c-43 0 -56 -4 -68 -22z M415 926 c-199 -63 -321 -258 -286 -457 31 -179 161 -309 340 -340 75 -14 171 1 248 37 116 55 209 188 220 314 l6 60 -49 0 -49 0 -17 -70 c-20 -84 -62 -147 -123 -188 -154 -102 -363 -44 -446 124 -35 72 -34 189 3 259 49 92 135 155 236 171 l42 7 0 48 0 49 -42 -1 c-24 0 -61 -6 -83 -13z M2020 882 l0 -50 43 -7 c99 -16 188 -80 235 -169 22 -43 27 -64 27 -131 0 -98 -23 -155 -90 -219 -177 -172 -471 -67 -511 183 l-7 41 -50 0 -50 0 6 -60 c11 -126 102 -257 218 -314 251 -123 542 26 590 303 39 221 -132 448 -351 468 l-60 6 0 -51z""")
        smiley.vertices -= smiley.vertices.mean(axis=0)

        draw_map(self.obstacles)
        theta = np.linspace(0, 2 * np.pi, 1000)
        # draw_gradient(robots[1].U) if params.num_robots>1 else draw_gradient(robots[0].U)
        for i, robot in enumerate(self.robots): 
            plt.plot(robot.sp[0], robot.sp[1], marker=smiley, color='blue', markersize=10, zorder=15) # robots poses
            plt.annotate(str(i), xy=(robot.sp[0], robot.sp[1]), xytext=(robot.sp[0], robot.sp[1]))

            self.draw_circle(robot, 'red')
        
        plt.plot(self.attack_robot.sp[0], self.attack_robot.sp[1], 'o', color='green', markersize=1, zorder=15)
        x = np.cos(theta) * self.params.drone_vel * 0.01 + self.attack_robot.sp[0]
        y = np.sin(theta) * self.params.drone_vel * 0.01 + self.attack_robot.sp[1]
        plt.plot(x, y, color='red')
        # plt.fill(x, y, color='green', alpha=0.3)
        # plt.gca().set_aspect('equal', adjustable='box')

        # robots_poses = []
        # for robot in robots: robots_poses.append(robot.sp)
        # robots_poses.sort(key=lambda p: atan2(p[1]-centroid[1],p[0]-centroid[0]))
        # plt.gca().add_patch( Polygon(robots_poses, color='yellow') )
        # plt.plot(centroid[0], centroid[1], '*', color='b', markersize=10) # label='Centroid position'
        # plt.plot(robot1.route[:,0], robot1.route[:,1], linewidth=2, color='green', label="Leader's path", zorder=10)
        # for robot in robots[1:]: plt.plot(robot.route[:,0], robot.route[:,1], '--', linewidth=2, color='red', zorder=10)
        plt.plot(P[:,0], P[:,1], linewidth=3, color='orange') # label='Global planner path'
        plt.plot(traj_global[sp_ind,0], traj_global[sp_ind,1], 'ro', color='blue', markersize=7) # label='Global planner setpoint'
        plt.plot(self.xy_start[0], self.xy_start[1],'bo',color='red', markersize=1) # label='start'
        plt.plot(self.xy_goal[0], self.xy_goal[1], 'bo',color='green', markersize=1) # label='goal'

        plt.text(-1, 1, str(self.time), fontdict=None)

        # ap = np.array(self.attack_point)
        # plt.plot(ap[:, 0], ap[:, 1], '*', color='grey', markersize=3)

        # if self.is_crash:
        #     plt.text(0, 0, "Crash", fontdict=None)
        
        # ------------------------------------------------------
        # for robot in metrics.robots:
        #     plt.plot(robot.route[:,0], robot.route[:,1], '--', label='drone %d' %robot.id, linewidth=2)
        # ------------------------------------------------------
        
        # plt.legend()
        plt.draw()

        plt.figure(2)
        plt.clf()
        plt.title("Robustness Change")
        x = [i for i in range(len(self.rob_list))]
        plt.fill_between(x, self.rob_list)

    def draw_circle(self, rbt: Robot, color='blue'):
        x_c, y_c = rbt.sp[0], rbt.sp[1]

        theta = np.linspace(0, 2*np.pi, 1000)
        r = self.params.influence_radius

        # 计算圆上的点的坐标
        x = np.cos(theta) * r + x_c
        y = np.sin(theta) * r + y_c

        # 绘制圆
        plt.plot(x, y, color=color)
        plt.fill(x, y, color='green', alpha=0.3)  # 填充圆的内部

    
    def get_info(self, robot: Robot) -> dict:
        info = dict()
        info['uav_pos'] = robot.sp
        info['obs_list'] = self.obstacles[:10]
        info['DIS_PRE_MIN_OBS'] = self.params.influence_radius
        info['time'] = self.t_array[-1]
        return info
    
    def get_rule2_info(self, robot_swarm: list()) -> dict:
        info = dict()
        info['uav_pos_list'] = [rbt.sp for rbt in robot_swarm]
        info['DIS_PRE_MIN_UAV'] = self.params.influence_radius
        info['DIS_PRE_MAX_UAV'] = self.params.influence_radius * 2
        info['time'] = self.t_array[-1]
        return info
    
    def move_obstacles(self):
        # small cubes movement
        self.obstacles[-3] += np.array([0.015, 0.0]) * self.params.drone_vel
        self.obstacles[-2] += np.array([-0.005, 0.005]) * self.params.drone_vel/2
        self.obstacles[-1] += np.array([0.0, 0.008]) * self.params.drone_vel/2

    def recode_res(self, message: str):
        """结果写入，保存图片

        Args:
            message (str): 结果信息
        """
        end_time = time.time()
        message += str(" 程序结束时间为: " + str(end_time - self.start_time))

        # 写入攻击结果信息
        with open(self.res_to_save_file_path, 'a') as f:
            f.write(self.attack_floder_name + ': ' + message)
        
        # 存储结果图片
        plt.savefig(self.res_pic_to_save_file_path)

        # 存储攻击点
        with open(self.attack_point_to_save_file_path, 'a') as f:
            f.write(str(self.attack_point))

    def copy_robot(self, rbt: Robot) -> Robot:
        copy_rbt = Robot(rbt.id, rbt.monitor)
        copy_rbt.sp = deepcopy(rbt.sp)
        copy_rbt.sp_global = deepcopy(rbt.sp_global)
        copy_rbt.route = deepcopy(rbt.route)
        copy_rbt.leader = rbt.leader
        return copy_rbt

    def build_attack_path(self):
        # 攻击机1s内允许的最长移动距离
        max_dis = 0.01 * self.params.drone_vel

        # 攻击路径列表
        attack_path: list[list] = list()
        for i, point in enumerate(self.attack_point):
            # 如果攻击路径列表为空，则添加一条包含初始点的攻击路径
            if len(attack_path) == 0:
                attack_path.append([point + [i]])
                continue
            
            # 攻击路径不为空，则遍历所有的攻击路径，找到一条最短的移动路径
            min_dis = math.inf
            min_dis_pos = -1
            for j, ap in enumerate(attack_path):
                dis = norm(np.array(point) - np.array(ap[-1][0: 2]))
                if (dis < max_dis * (i - ap[-1][2])) and dis < min_dis:
                    min_dis = dis
                    min_dis_pos = j
            
            # 如果找到路径且两点之间的路径小于允许的最长移动距离，则归到该攻击路径下
            if not min_dis_pos == -1:
                attack_path[min_dis_pos].append(point + [i])
            else:
                attack_path.append([point + [i]]) # 找不到路径，添加一条全新的攻击路径

        plt.figure(figsize=(10, 10))
        plt.title("Attack Path")
        for ap in attack_path:
            ap_n = np.array(ap)
            plt.plot(ap_n[:, 0], ap_n[:, 1], '*', color='black', markersize=3)
            plt.plot(ap_n[:, 0], ap_n[:, 1])
        plt.savefig(self.attack_path_pic_to_save_file_path)

        with open(self.attack_path_data_to_save_file_path, 'a') as f:
            f.write(str(attack_path))

    def draw_rob_change_pic(self):
        plt.figure(figsize=(10, 10))
        plt.title("Robustness Change")
        x = [i for i in range(len(self.rob_list))]
        plt.fill_between(x, self.rob_list)
        plt.savefig(self.rob_change_pic_to_save_file_path)

if __name__ == '__main__':
    for _ in range(20):
        attack_demo = attack(attack_strategy=3)
        attack_demo.start()