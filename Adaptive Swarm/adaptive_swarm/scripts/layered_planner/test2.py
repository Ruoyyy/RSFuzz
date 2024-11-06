import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
import time
import math
from numpy.linalg import norm

# 创建一个表示圆的角度数组
theta = np.linspace(0, 2*np.pi, 1000)

def draw_circle(x_c, y_c, color='blue'):
    r = 0.15

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


    sub_r = r / count * ((2 * (count - 1) + 1) / 2)
    for j in range(count2):
        x = np.cos(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + x_c
        y = np.sin(2 * np.pi / count2 * ((1 + 2 * j) / 2)) * sub_r + y_c
        plt.plot(x, y, color=color, marker='o')

def worker(i: int):
    print("i am i worker" + str(i))
    time.sleep(3)

def read_attack_point_file() -> list:
    attack_point = None
    with open('/home/czh/attack/adaptive_swarm/scripts/layered_planner/data_collection/attack_res/2023-12-11-10-35-09/attack_path.txt', 'r') as f:
        attack_point = eval(f.readline())
    return attack_point

def build_path_1(attack_point: list):
    # 每个点之间允许的最长移动距离
    max_dis = 0.01 * 4.0
    # 攻击路径列表
    attack_path: list[list] = list()
    for i, point in enumerate(attack_point):
        # 如果攻击路径列表为空，则添加一条包含初始点的攻击路径
        if len(attack_path) == 0:
            attack_path.append([point])
            continue
        
        # 攻击路径不为空，则遍历所有的攻击路径，找到一条最短的移动路径
        min_dis = math.inf
        min_dis_pos = -1
        for j, ap in enumerate(attack_path):
            dis = norm(np.array(point) - np.array(ap[-1]))
            if dis < min_dis:
                min_dis = dis
                min_dis_pos = j
        
        # 如果找到路径且两点之间的路径小于允许的最长移动距离，则归到该攻击路径下
        if (min_dis < max_dis) and (not min_dis_pos == -1):
            attack_path[min_dis_pos].append(point)
        else:
            attack_path.append([point]) # 找不到路径，添加一条全新的攻击路径
    
    # 画图
    ap = np.array(attack_point)
    plt.figure(figsize=(5, 5))
    plt.title("Attack Point")
    plt.plot(ap[:, 0], ap[:, 1], '*', color='black', markersize=3)

    for i, ap in enumerate(attack_point):
        plt.annotate(str(i), xy=(attack_point[i][0], attack_point[i][1]), xytext=(attack_point[i][0], attack_point[i][1]))

    plt.figure(figsize=(5, 5))
    plt.title("Attack Path")
    for ap in attack_path:
        ap_n = np.array(ap)
        plt.plot(ap_n[:, 0], ap_n[:, 1], '*', color='black', markersize=3)
        plt.plot(ap_n[:, 0], ap_n[:, 1])
    
    plt.show()

def build_path_2(attack_point: list):
    # 每个点之间允许的最长移动距离
    max_dis = 0.01 * 4.0
    # 攻击路径列表
    attack_path: list[list] = list()
    for i, point in enumerate(attack_point):
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
    
    # 画图
    ap = np.array(attack_point)
    plt.figure(figsize=(5, 5))
    plt.title("Attack Point")
    plt.plot(ap[:, 0], ap[:, 1], '*', color='black', markersize=3)

    for i, ap in enumerate(attack_point):
        plt.annotate(str(i), xy=(attack_point[i][0], attack_point[i][1]), xytext=(attack_point[i][0], attack_point[i][1]))

    plt.figure(figsize=(5, 5))
    plt.title("Attack Path")
    for ap in attack_path:
        ap_n = np.array(ap)
        plt.plot(ap_n[:, 0], ap_n[:, 1], '*', color='black', markersize=3)
        plt.plot(ap_n[:, 0], ap_n[:, 1])
    
    plt.show()

if __name__ == '__main__':
    # draw_circle(1.2, 1.0)
    # draw_circle(1.3, 1.1)
    # # draw_circle(1.35, 1.25980762, 'red')
    # # draw_circle(1.05, 1.25980762, 'orange')
    # # draw_circle(1.2, 1.51961524, 'yellow')
    # plt.gca().set_aspect('equal', adjustable='box')
    # plt.title('Full Circle')
    # plt.show()

    # grid = np.zeros((10, 10))
    # grid[1:2, 2:3] += 1
    # print(grid)

    # print(time.localtime)
    # threads = []
    # for i in range(5):
    #     thread = Thread(target=worker, kwargs={
    #         "i": i
    #     })
    #     threads.append(thread)
    #     thread.start()
    # for thread in threads:
    #     thread.join()
    # print("ok")
    # print(time.localtime)
    # l = [[0, 1], [1, 2]]
    # n = np.array(l)
    # print(n[:, 0])
    # print(n)

    # build_path_1(read_attack_point_file())

    # 创建一个窗口和两个子图
    # fig, (ax1, ax2) = plt.subplots(2, 1)

    fig1 = plt.figure(1)
    fig2 = plt.figure(2)

    # # 模拟的数据
    # x = np.linspace(0, 2*np.pi, 100)
    # y1 = np.sin(x)
    # y2 = np.cos(x)

    data_list = list()

    # 开始绘制循环
    for i in range(100):
        # 更新数据
        # y1 = y1 * 0.99
        # y2 = y2 * 0.99
        fig1 = plt.figure(1)

        data_list.append(i)

        # 清除旧图表
        plt.clf()

        # 绘制新图表
        plt.plot(data_list, data_list)

        # 设置图表标题
        plt.title("Damping Sine Wave")

        # 暂停一会儿，以便更新图表
        plt.pause(0.1)

    # 显示图表
    plt.show()
