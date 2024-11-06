from tool.monitor import Monitor

if __name__ == '__main__':
    raw_data = {
        'uva_pos_x': 3.0,
        'uva_pos_y': 3.0,
        'obs_pos_x': 4.0,
        'obs_pos_y': 4.0,
        'DIS_PRE_MIN_OBS': 1
    }
    monitor = Monitor(raw_data=raw_data, specification=None)
    rob = monitor.continuous_monitor()
    print(rob)
