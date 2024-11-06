import numpy as np
from tool import *

def test_monitor():
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

def test_get_obs_pos():
    obs = np.array([[0.5, 0], [2.5, 0.], [2.5, 0.3], [0.5, 0.3]])
    obs = np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]])
    obs = np.array([[-2, -2], [-0.5, -2], [-0.5, -1.8], [-2, -1.8]])
    obs = np.array([[-0.7, -1.8], [-0.5, -1.8], [-0.5, -0.8], [-0.7, -0.8]])
    print(get_obs_pos(obs))

def test_obs():
    obstacles = [
        # bugtrap
        np.array([[0.5, 0], [2.5, 0.], [2.5, 0.3], [0.5, 0.3]]),
        np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]),
        
        np.array([[-2, -2], [-0.5, -2], [-0.5, -1.8], [-2, -1.8]]),
        np.array([[-0.7, -1.8], [-0.5, -1.8], [-0.5, -0.8], [-0.7, -0.8]]),

        # walls
        np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]), # comment this for better 3D visualization
        np.array([[-2.5, 2.47], [2.5, 2.47], [2.5, 2.5], [-2.5, 2.5]]),
        np.array([[-2.5, -2.47], [-2.47, -2.47], [-2.47, 2.47], [-2.5, 2.47]]),
        np.array([[2.47, -2.47], [2.5, -2.47], [2.5, 2.47], [2.47, 2.47]]), # comment this for better 3D visualization
    ]
    print(obstacles[:-4])

if __name__ == '__main__':
    test_obs()