import rtamt
import sys
import math
from collections import deque
import numpy as np
import logging
from time import strftime, localtime
from numpy.linalg import norm

class Monitor:
    def __init__(self, name, create_time) -> None:
        self.name = name
        self.rule_obj_list = dict()
        self.pos_dq = deque()
        dq_len = 5
        self.logger = Logger(name=name, create_time=create_time)
        # for _ in range(dq_len):
        #     self.pos_dq.append((self.raw_data['uav_pos_x'], self.raw_data['uav_pos_y']))

        self.rule_init()

    def rule_init(self) -> None:
        """规则初始化
        """
        self.rule_obj_list['rule_obj']: list[dict] = list()
        self.rule_prepare()
        self.logger.debug("初始化后rule_obj_list内容: %s" % (self.rule_obj_list))

    def data_prepare(self) -> None:
        """数据预处理

        rule_obj_list: 规则对象列表
        rule_obj_list = {
            'time': time,
            'rule_obj': [
                rule_obj_1,
                rule_obj_2,
                ...
            ]
        }
        """
        self.rule_obj_list['time'] = self.raw_data['time']
        # 规则数据处理
        self.rule1_data_prepare()
        self.rule2_data_prepare()
        self.rule3_data_prepare()
        self.rule4_data_prepare()
    
    def rule1_data_prepare(self, raw_data) -> None:
        """规则一数据处理
        """
        # uav_obs_dis = math.sqrt((self.raw_data['uav_pos_x'] - self.raw_data['obs_pos_x']) ** 2 + (self.raw_data['uav_pos_y'] - self.raw_data['obs_pos_y']) ** 2)
        # uav_obs_dis = point_to_rectangle(self.raw_data['uav_pos'], self.raw_data['obs_pos'])
        uav_obs_dis, min_obs = nearest_obs(raw_data['uav_pos'][0], raw_data['uav_pos'][1], raw_data['obs_list'])
        self.logger.debug("最近障碍物为: %s, 最近距离为%s" % (min_obs, uav_obs_dis))
        self.rule_obj_list['rule_obj'][0]['rule_var_obj'][0]['rule_var_value'] = uav_obs_dis
        self.rule_obj_list['rule_obj'][0]['rule_var_obj'][1]['rule_var_value'] = raw_data['DIS_PRE_MIN_OBS']

    def rule2_data_prepare(self, raw_data) -> None:
        """规则二数据处理
        """
        # ego_follow_dis = math.sqrt((self.raw_data['uav_pos'][0] - self.raw_data['other_uav_pos'][0]) ** 2 + (self.raw_data['uav_pos'][1] - self.raw_data['other_uav_pos'][1] ** 2))
        ego_follow_dis = norm(raw_data['uav_pos'] - raw_data['other_uav_pos'])
        self.logger.debug("两点距离为: %s" % (ego_follow_dis))
        self.rule_obj_list['rule_obj'][1]['rule_var_obj'][0]['rule_var_value'] = ego_follow_dis
        self.rule_obj_list['rule_obj'][1]['rule_var_obj'][1]['rule_var_value'] = raw_data['DIS_PRE_MIN_UAV']
        self.rule_obj_list['rule_obj'][1]['rule_var_obj'][2]['rule_var_value'] = raw_data['DIS_PRE_MAX_UAV']

    def rule3_data_prepare(self) -> None:
        """规则三数据处理
        """
        self.rule_obj_list['rule_obj'][2]['rule_var_obj'][0]['rule_var_value'] = self.raw_data['time']
        self.rule_obj_list['rule_obj'][2]['rule_var_obj'][1]['rule_var_value'] = self.raw_data['task_finish_time']
        self.rule_obj_list['rule_obj'][2]['rule_var_obj'][2]['rule_var_value'] = self.raw_data['PRE_TIME_THRESHOLDS']

    def rule4_data_prepare(self) -> None:
        """规则四数据处理
        """
        # self.pos_dq.append((self.raw_data['uav_pos_x'], self.raw_data['uav_pos_y']))
        pass

    def rule_prepare(self) -> None:
        """解析所有规则，并将其生成一个规则对象，放入到rule_obj_list中
        """
        self.rule1_prepare()
        self.rule2_prepare()
        # self.rule3_prepare()
        # self.rule4_prepare()

    def rule1_prepare(self) -> None:
        """规则一：避免和障碍物碰撞
        """
        spec_context = 'uav_obs_dis >= DIS_PRE_MIN_OBS'
        rule_obj = {
            'rule': spec_context,
            'rule_var_obj': [
                {
                    'rule_var_name': 'uav_obs_dis',
                    'rule_var_type': 'float',
                    'rule_var_value': 0.0
                },
                {
                    'rule_var_name': 'DIS_PRE_MIN_OBS',
                    'rule_var_type': 'float',
                    'rule_var_value': 0.0
                }
            ]
        }
        # spec = rtamt.STLDiscreteTimeSpecification()
        spec = rtamt.StlDiscreteTimeSpecification()
        for var_obj in rule_obj['rule_var_obj']:
            spec.declare_var(var_obj['rule_var_name'], var_obj['rule_var_type'])
        spec.spec = rule_obj['rule']
        rule_obj['spec'] = spec
        self.rule_obj_list['rule_obj'].append(rule_obj)

    def rule2_prepare(self) -> None:
        """规则二：和其他无人机保持距离
        """
        spec = '(ego_follow_dis >= DIS_PRE_MIN_UAV) & (ego_follow_dis <= DIS_PRE_MAX_UAV)'
        rule_obj = {
            'rule': spec,
            'rule_var_obj': [
                {
                    'rule_var_name': 'ego_follow_dis',
                    'rule_var_type': 'float',
                    'rule_var_value': 0.0
                },
                {
                    'rule_var_name': 'DIS_PRE_MIN_UAV',
                    'rule_var_type': 'float',
                    'rule_var_value': 0.0
                },
                {
                    'rule_var_name': 'DIS_PRE_MAX_UAV',
                    'rule_var_type': 'float',
                    'rule_var_value': 0.0
                }
            ]
        }
        # spec = rtamt.STLDiscreteTimeSpecification()
        spec = rtamt.StlDiscreteTimeSpecification()
        for var_obj in rule_obj['rule_var_obj']:
            spec.declare_var(var_obj['rule_var_name'], var_obj['rule_var_type'])
        spec.spec = rule_obj['rule']
        rule_obj['spec'] = spec
        self.rule_obj_list['rule_obj'].append(rule_obj)

    def rule3_prepare(self) -> None:
        """规则三：保证任务完成时间
        """
        spec = 'cur_time - task_finish_time <= PRE_TIME_THRESHOLDS'
        rule_obj = {
            'rule': spec,
            'rule_var_obj': [
                {
                    'rule_var_name': 'cur_time',
                    'rule_var_type': 'float',
                    'rule_var_value': 0.0
                },
                {
                    'rule_var_name': 'task_finish_time',
                    'rule_var_type': 'float',
                    'rule_var_value': 0.0
                },
                {
                    'rule_var_name': 'PRE_TIME_THRESHOLDS',
                    'rule_var_type': 'float',
                    'rule_var_value': 0.0
                }
            ]
        }
        spec = rtamt.STLDenseTimeSpecification()
        for var_obj in rule_obj['rule_var_obj']:
            spec.declare_var(var_obj['rule_var_name'], var_obj['rule_var_type'])
        spec.spec = rule_obj['rule']
        rule_obj['spec'] = spec
        self.rule_obj_list['rule_obj'].append(rule_obj)

    def rule4_prepare(self) -> None:
        """规则四：确保无人机集群是朝着任务终点的方向移动
        """
        before_k_pos = self.pos_dq.popleft()
        uva_dir_x = self.raw_data['uav_pos_x'] - before_k_pos[0]
        uva_dir_y = self.raw_data['uav_pos_y'] - before_k_pos[1]

    def single_rule_monitor(self, rule_obj) -> float:
        """单个规则监测

        Args:
            rule_obj (dict): 规则对象，包含对象规则、变量和变量类型
            rule_obj = {
                spec: spec,
                rule: string,
                rule_var_obj: [
                    {
                        rule_var_name: string,
                        rule_var_type: string,
                        rule_var_vaule: rule_var_type
                    }
                    ...
                ]
            }

        Returns:
            float: 单个规则健壮性得分
        """
        try:
            rule_obj['spec'].parse()
            rule_obj['spec'].pastify()
        except rtamt.STLParseException as err:
            print('STL Parse Exception: {}'.format(err))
            sys.exit()

        update_data = list()
        for var_obj in rule_obj['rule_var_obj']:
            update_data.append((
                var_obj['rule_var_name'], var_obj['rule_var_value']
            ))

        rob = rule_obj['spec'].update(self.rule_obj_list['time'], update_data)
        self.logger.debug("规则%s实时监测数据为: %s" % (rule_obj['rule'], rule_obj))
        self.logger.debug("规则%s健壮性为: %s" % (rule_obj['rule'], rob))
        return rob
    
    def multi_rule_monitor(self, raw_data) -> float:
        """所有规则监测

        Args:
            raw_data (_type_): 原生数据

        Returns:
            float: 所有规则健壮性得分总和
        """
        self.raw_data = raw_data
        self.data_prepare()
        total_rob = 0.0
        for rule_obj in self.rule_obj_list['rule_obj']:
            single_rob = self.single_rule_monitor(rule_obj)
            total_rob += single_rob
        self.logger.debug("规则监测实时数据为：%s" % (self.rule_obj_list))
        self.logger.debug("全部规则健壮性之和为：%s" % (total_rob))
        return total_rob

    def rule1_monitor(self, raw_data) -> float:
        """单规则一检测

        Args:
            raw_data (dict): 原生数据

        Returns:
            float: 规则健壮性
        """
        self.rule_obj_list['time'] = raw_data['time']
        self.rule1_data_prepare(raw_data)
        rule_rob = self.single_rule_monitor(self.rule_obj_list['rule_obj'][0])
        return rule_rob

    def rule2_monitor(self, raw_data) -> float:
        """单规则二检测

        Args:
            raw_data (dict): 原生数据

        Returns:
            float: 规则健壮性
        """
        self.rule_obj_list['time'] = raw_data['time']
        self.rule2_data_prepare(raw_data)
        rule_rob = self.single_rule_monitor(self.rule_obj_list['rule_obj'][1])
        return rule_rob
    
    def rule3_monitor(self, raw_data) -> float:
        """单规则三检测

        Args:
            raw_data (dict): 原生数据

        Returns:
            float: 规则健壮性
        """
        self.raw_data = raw_data
        self.rule_obj_list['time'] = self.raw_data['time']
        self.rule3_data_prepare()
        rule_rob = self.single_rule_monitor(self.rule_obj_list['rule_obj'][3])
        return rule_rob
    
    def rule4_monitor(self, raw_data) -> float:
        """单规则四检测

        Args:
            raw_data (dict): 原生数据

        Returns:
            float: 规则健壮性
        """
        self.raw_data = raw_data
        self.rule_obj_list['time'] = self.raw_data['time']
        self.rule4_data_prepare()
        rule_rob = self.single_rule_monitor(self.rule_obj_list['rule_obj'][4])
        return rule_rob

    def multi_rule2_monitor(self, raw_data) -> float:
        """多机-规则二检测

        Args:
            raw_data (dict): 原生数据

        Returns:
            float: 规则健壮性
        """
        self.rule_obj_list['time'] = raw_data['time']
        total_rob = 0.0
        for i in range(len(raw_data['uav_pos_list'])):
            for j in range(i+1, len(raw_data['uav_pos_list'])):
                raw_data['uav_pos'] = raw_data['uav_pos_list'][i]
                raw_data['other_uav_pos'] = raw_data['uav_pos_list'][j]
                self.rule2_data_prepare(raw_data)
                total_rob += self.single_rule_monitor(self.rule_obj_list['rule_obj'][1])
        return total_rob

def point_to_point(x1: float, y1: float, x2: float, y2: float) -> float:
    """计算两点之间的距离

    Args:
        x1 (float): 点1的x坐标
        y1 (float): 点1的y坐标
        x2 (float): 点2的x坐标
        y2 (float): 点2的y坐标

    Returns:
        float: 两点之间的距离
    """
    lineMagnitude = math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
    return lineMagnitude

def point_to_line(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """计算点P到线AB的距离

    Args:
        px (float): 点p的x坐标
        py (float): 点p的x坐标
        x1 (float): 点A的x坐标
        y1 (float): 点A的y坐标
        x2 (float): 点B的x坐标
        y2 (float): 点B的y坐标

    Returns:
        float: 点到线的距离
    """
    line_magnitude = point_to_point(x1, y1, x2, y2)

    # 如果线段距离很近，直接返回P到A的距离
    if line_magnitude < 0.00000001:
        return point_to_point(px, py, x1, y1)
    
    # 向量AB·向量AP
    u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
    # 向量AP在向量AB方向的投影与向量AB模的比值
    u = u1 / (line_magnitude * line_magnitude)

    # 点到直线的投影不在线段内, 计算点到两个端点距离的最小值即为"点到线段最小距离" 
    if (u < 0) or (u > 1):
        return min(point_to_point(px, py, x1, y1), point_to_point(px, py, x2, y2))
    
    # 投影点在线段内部, 计算方式同点到直线距离, u为投影点距离x1在x1x2上的比例, 以此计算出投影点的坐标
    ix = x1 + u * (x2 - x1)
    iy = y1 + u * (y2 - y1)
    return point_to_point(px, py, ix, iy)

def point_to_rectangle(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """计算点P(px,py)到垂直矩形A(x1,y1)B(x2,y2)之间的最短距离

    Args:
        px (float): 点p的x坐标
        py (float): 点p的x坐标
        x1 (float): 点A的x坐标
        y1 (float): 点A的y坐标
        x2 (float): 点B的x坐标
        y2 (float): 点B的y坐标

    Returns:
        float: 点P(px,py)到垂直矩形A(x1,y1)B(x2,y2)之间的最短距离
    """
    # 点在矩形内
    if px > min(x1,x2) and px < max(x1,x2) and py > min(y1,y2) and py < max(y1,y2):
        return -math.inf
    distance1 = point_to_line(px, py, x1, y1, x1, y2)
    distance2 = point_to_line(px, py, x1, y1, x2, y1)
    distance3 = point_to_line(px, py, x2, y2, x1, y2)
    distance4 = point_to_line(px, py, x2, y2, x2, y1)
    return min(distance1, distance2, distance3, distance4)

def nearest_obs(px: float, py: float, obs_list: np.array) -> float:
    """计算离点P最近的矩形障碍物的距离

    Args:
        px (float): 点P的x坐标
        py (float): 点P的y坐标
        obs_list (list): 障碍物列表

    Returns:
        float: 离点P最近的矩形障碍物的距离
    """
    min_dis = math.inf
    min_obs = None
    for obs in obs_list:
        obs_pos = get_obs_pos(obs)
        obs_dis = point_to_rectangle(px, py, obs_pos[0][0], obs_pos[0][1], obs_pos[1][0], obs_pos[1][1])
        if obs_dis < min_dis:
            min_obs = obs_pos
            min_dis = obs_dis
    return min_dis, min_obs

def get_obs_pos(obs: np.array) -> np.array:
    """获取一个矩形障碍物的左上角和右下角坐标

    Args:
        obs (np.array): 障碍物

    Returns:
        np.array: 障碍物的左上角和右下角坐标
    """
    left, right, bottom, top = obs[0][0], obs[0][0], obs[0][1], obs[0][1]
    for i in range(3):
        left = min(left, obs[i+1][0])
        right = max(right, obs[i+1][0])
        bottom = min(bottom, obs[i+1][1])
        top = max(top, obs[i+1][1])
    return np.array([left, top]), np.array([right, bottom])

class Logger:
    def __init__(self, name: str, create_time) -> None:
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)

        if not self.logger.handlers:
            uuid_str = strftime("%Y-%m-%d-%H-%M-%S", create_time) 
            tmp_file_name ='%s.log' % uuid_str

            self.fh = logging.FileHandler('/media/ruoyu/ESD-USB/论文/attack1/attack/log_file/%s' % tmp_file_name)
            self.fh.setLevel(logging.DEBUG)

            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            self.fh.setFormatter(formatter)

            self.logger.addHandler(self.fh)
    
    def info(self, message):
        self.logger.info(message)
    
    def debug(self, message):
        self.logger.debug(message)

    def warning(self, message):
        self.logger.warning(message)

    def error(self, message):
        self.logger.error(message)

    def critical(self, message):
        self.logger.critical(message)

if __name__ == '__main__':

    monitor = Monitor("test", localtime())

    # raw_data = {
    #     'time': 0,
    #     'uav_pos': np.array([1, 1]),
    #     'obs_list': [
    #         np.array([[0.5, 0], [2.5, 0.], [2.5, 0.3], [0.5, 0.3]]),
    #         np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]),
    #         np.array([[-2, -2], [-0.5, -2], [-0.5, -1.8], [-2, -1.8]]),
    #         np.array([[-0.7, -1.8], [-0.5, -1.8], [-0.5, -0.8], [-0.7, -0.8]]),
    #     ],
    #     'DIS_PRE_MIN_OBS': 0.15,
    # }
    # rob = monitor.rule1_monitor(raw_data)
    # print(rob)

    # raw_data = {
    #     'time': 0,
    #     'uav_pos': np.array([1, 1]),
    #     'other_uav_pos': np.array([2, 2]),
    #     'DIS_PRE_MIN_UAV': 1,
    #     'DIS_PRE_MAX_UAV': 2,
    # }
    # rob = monitor.rule2_monitor(raw_data)
    # print(rob)

    raw_data = {
        'time': 0,
        'uav_pos_list': [
            np.array([1, 1]),
            np.array([2, 2]),
            np.array([3, 3]),
            np.array([4, 4]),
        ],
        'DIS_PRE_MIN_UAV': 1,
        'DIS_PRE_MAX_UAV': 2,
    }
    rob = monitor.multi_rule2_monitor(raw_data)
    print(rob)
