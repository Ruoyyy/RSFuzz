import rtamt
import sys
import copy
import math
from collections import deque
from pprint import pprint
# from tool.log_tool import Logger
from log_tool import Logger
import numpy as np

class Monitor:
    def __init__(self, name=None) -> None:
        self.name = name
        self.rule_obj_list = dict()
        self.pos_dq = deque()
        dq_len = 5
        self.logger = Logger(name=name)
        # for _ in range(dq_len):
        #     self.pos_dq.append((self.raw_data['uav_pos_x'], self.raw_data['uav_pos_y']))

        self.rule_init()

    def rule_init(self):
        """规则初始化
        """
        self.data_prepare()
        self.logger.debug("初始化后rule_obj_list内容: %s" % (self.rule_obj_list))
        for rule_obj in self.rule_obj_list['rule_obj']:
            self.single_rule_monitor(rule_obj=rule_obj)

    def data_prepare(self, raw_data=None) -> None:
        """数据和规则预处理

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
        if not raw_data is None:
            self.raw_data = raw_data
            self.rule_obj_list['time'] = self.raw_data['time']
            # 规则数据处理
            self.rule1_data_prepare()
            # self.rule2_data_prepare()
            # self.rule3_data_prepare()
            # self.rule4_data_prepare()
        else:
            # 初始化
            self.rule_obj_list['time'] = 0
            self.rule_obj_list['rule_obj'] = list()
            self.rule_prepare()
    
    def rule1_data_prepare(self) -> None:
        """规则一数据处理
        """
        # uav_obs_dis = math.sqrt((self.raw_data['uav_pos_x'] - self.raw_data['obs_pos_x']) ** 2 + (self.raw_data['uav_pos_y'] - self.raw_data['obs_pos_y']) ** 2)
        uav_obs_dis = point_to_rectangle(self.raw_data['uav_pos'], self.raw_data['obs_pos'])
        self.rule_obj_list['rule_obj'][0]['rule_var_obj'][0]['rule_var_value'] = uav_obs_dis
        self.rule_obj_list['rule_obj'][0]['rule_var_obj'][1]['rule_var_value'] = self.raw_data['DIS_PRE_MIN_OBS']

    def rule2_data_prepare(self) -> None:
        """规则二数据处理
        """
        ego_follow_dis = math.sqrt((self.raw_data['ego_pos_x'] - self.raw_data['follow_pos_x']) ** 2 + (self.raw_data['ego_pos_y'] - self.raw_data['follow_pos_y']) ** 2)
        self.rule_obj_list['rule_obj'][1]['rule_var_obj'][0]['rule_var_value'] = ego_follow_dis
        self.rule_obj_list['rule_obj'][1]['rule_var_obj'][1]['rule_var_value'] = self.raw_data['DIS_PRE_MIN_UAV']
        self.rule_obj_list['rule_obj'][1]['rule_var_obj'][2]['rule_var_value'] = self.raw_data['DIS_PRE_MAX_UAV']

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
        # self.rule2_prepare()
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
        spec = rtamt.STLDiscreteTimeSpecification()
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
        spec = rtamt.STLDenseTimeSpecification()
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
        self.data_prepare(raw_data=raw_data)
        total_rob = 0.0
        for rule_obj in self.rule_obj_list['rule_obj']:
            single_rob = self.single_rule_monitor(rule_obj)
            total_rob += single_rob
        self.logger.debug("规则监测实时数据为：%s" % (self.rule_obj_list))
        self.logger.debug("全部规则健壮性之和为：%s" % (total_rob))
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
    """计算点到线的距离

    Args:
        px (float): 点p的x坐标
        py (float): 点p的x坐标
        x1 (float): 线一端的x坐标
        y1 (float): 线一端的y坐标
        x2 (float): 线另一端的x坐标
        y2 (float): 线另一端的y坐标

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

# 计算点P(px,py)到垂直矩形A(x1,y1)B(x2,y2)之间的最短距离
def point_to_rectangle(uav_pos, obs_pos):
    print(obs_pos[0])
    px, py = uav_pos[0], uav_pos[1]
    x1, y1, x2, y2 = obs_pos[0][0], obs_pos[0][1], obs_pos[1][0], obs_pos[1][1]
    if px > min(x1,x2) and px < max(x1,x2) and py > min(y1,y2) and py < max(y1,y2):
        #点在矩形内
        return -100000000000
    distance1 = point_to_line(px, py, x1, y1, x1, y2)
    distance2 = point_to_line(px, py, x1, y1, x2, y1)
    distance3 = point_to_line(px, py, x2, y2, x1, y2)
    distance4 = point_to_line(px, py, x2, y2, x2, y1)
    return min(distance1,distance2,distance3,distance4)
        
if __name__ == '__main__':
    raw_data = {
        # 'uav_pos_x': 3.0,
        # 'uav_pos_y': 3.0,
        # 'obs_pos_x': 4.0,
        # 'obs_pos_y': 4.0,
        'DIS_PRE_MIN_OBS': 1,
        'uav_pos': np.array([0, 0]),
        # 'obs_pos': np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]),
        'obs_pos': np.array([[0.5, 1.5], [0.8, 0.3]]),
        # 'ego_pos_x': 1.0,
        # 'ego_pos_y': 1.0,
        # 'follow_pos_x': 1.0,
        # 'follow_pos_y': 1.0,
        # 'DIS_PRE_MIN_UAV': 1.0,
        # 'DIS_PRE_MAX_UAV': 1.0,
        # 'task_finish_time': 300.0,
        # 'PRE_TIME_THRESHOLDS': 5.0,
        'time': 1.0,
    }
    monitor = Monitor()
    rob = monitor.multi_rule_monitor(raw_data=raw_data)