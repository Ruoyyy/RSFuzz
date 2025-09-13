import matplotlib.pyplot as plt

obstacles = [
    [(0.5, 2.5),(0.8, 2.5),(0.8, 2.0),(0.5, 2.0)],
    [(-1.0, 2.5), (-0.7, 2.5), (-0.7, 1.8), (-1.0, 1.8)],
    [(-1.3, 0.6), (-0.7, 0.6), (-0.7, 0.3), (2.5, 0.3), (2.5, 0), (-1 ,0), (-1.0, 0.3), (-1.3, 0.3)],
    [(-2.0, -1.3),(-1.4, -1.3),(-1.4,-2.5),(-2.0,-2.5)],
    [(0,-1.6),(0.3,-1.6),(0.3,-2.5),(0,-2.5)]
]

# 画出障碍物
for obstacle in obstacles:
    x = [point[0] for point in obstacle]  # 获取障碍物顶点的x坐标
    y = [point[1] for point in obstacle]  # 获取障碍物顶点的y坐标
    plt.plot(x + [x[0]], y + [y[0]], color='blue', linewidth=2)  # 最后一个点与起点连线，形成封闭多边形
    plt.fill(x, y, color='blue', alpha=0.5)  # 使用fill函数填充障碍物区域
plt.scatter(1.2, 1.25, c='black', s=50, alpha=0.7, marker='o',label=f'({1.2}, {1.25})')
plt.scatter(2.2, -2.2, c='red', s=50, alpha=0.7, marker='*',label=f'({2.2}, {-2.2})')
# 调整坐标轴标签字体大小和样式
plt.xticks(fontsize=8, fontname='Times New Roman')
plt.yticks(fontsize=8, fontname='Times New Roman')

# 显示自定义图例
plt.legend(loc='best', bbox_to_anchor=(1, 0.5), prop={'size': 8, 'family': 'Times New Roman'})
# 将坐标轴起点设置为0，并设置范围
plt.gca().set_xlim([-2.5, 2.5])  # 设置x轴范围从0到map_size
plt.gca().set_ylim([-2.5, 2.5])  # 设置y轴范围从0到map_size
plt.gca().set_aspect('equal', adjustable='box')  # 保持坐标轴比例一致
# 调整坐标轴朝内
plt.tick_params(direction='in')
plt.savefig('A1map.pdf', format='pdf')
plt.show()
