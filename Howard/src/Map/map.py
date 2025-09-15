import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# 创建一个新的三维图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# 设置背景颜色
ax.set_facecolor('white')
fig.patch.set_facecolor('white')
# 绘制长宽高范围皆是[35，45]的正方体
cube_min = [35, 35, 35]
cube_max = [45, 45, 45]
verts = [
    [cube_min, [cube_min[0], cube_max[1], cube_min[2]], [cube_min[0], cube_max[1], cube_max[2]], [cube_min[0], cube_min[1], cube_max[2]]],
    [cube_min, [cube_max[0], cube_min[1], cube_min[2]], [cube_max[0], cube_min[1], cube_max[2]], [cube_min[0], cube_min[1], cube_max[2]]],
    [cube_max, [cube_max[0], cube_min[1], cube_max[2]], [cube_max[0], cube_min[1], cube_min[2]], [cube_max[0], cube_max[1], cube_min[2]]],
    [cube_max, [cube_min[0], cube_max[1], cube_max[2]], [cube_min[0], cube_max[1], cube_min[2]], [cube_min[0], cube_min[1], cube_min[2]]],
    [cube_min, [cube_max[0], cube_min[1], cube_min[2]], [cube_max[0], cube_max[1], cube_min[2]], [cube_min[0], cube_max[1], cube_min[2]]],
    [cube_max, [cube_max[0], cube_min[1], cube_max[2]], [cube_min[0], cube_min[1], cube_max[2]], [cube_min[0], cube_max[1], cube_max[2]]]
]
cube = Poly3DCollection(verts, alpha=0.2, facecolor='green')
ax.add_collection3d(cube)

# 绘制半径为4的球
spheres = [
    {'center': [10, 30, 0], 'radius': 4},
    {'center': [10, 50, 10], 'radius': 4},
    {'center': [20, 50, 25], 'radius': 4}
]
for sphere in spheres:
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = sphere['center'][0] + sphere['radius'] * np.cos(u) * np.sin(v)
    y = sphere['center'][1] + sphere['radius'] * np.sin(u) * np.sin(v)
    z = sphere['center'][2] + sphere['radius'] * np.cos(v)
    ax.plot_surface(x, y, z, color='red', alpha=0.5)

# 绘制坐标为（15，50，20）、（0，50，0）、（5，5，5）的三个点
points = [
    [15, 50, 20],
    [0, 50, 0],
    [5, 5, 5]
]
markers = ['o', 's', '*']
labels = ['(15, 50, 20)', '(0, 50, 0)', '(5, 5, 5)']

for point, marker, label in zip(points, markers, labels):
    ax.scatter(point[0], point[1], point[2], c='b', marker=marker, s=50,alpha=0.7, label=label)

# 绘制长宽高皆为[0，10]的正方体
small_cube_min = [0, 0, 0]
small_cube_max = [10, 10, 10]
verts_small = [
    [small_cube_min, [small_cube_min[0], small_cube_max[1], small_cube_min[2]], [small_cube_min[0], small_cube_max[1], small_cube_max[2]], [small_cube_min[0], small_cube_min[1], small_cube_max[2]]],
    [small_cube_min, [small_cube_max[0], small_cube_min[1], small_cube_min[2]], [small_cube_max[0], small_cube_min[1], small_cube_max[2]], [small_cube_min[0], small_cube_min[1], small_cube_max[2]]],
    [small_cube_max, [small_cube_max[0], small_cube_min[1], small_cube_max[2]], [small_cube_max[0], small_cube_min[1], small_cube_min[2]], [small_cube_max[0], small_cube_max[1], small_cube_min[2]]],
    [small_cube_max, [small_cube_min[0], small_cube_max[1], small_cube_max[2]], [small_cube_min[0], small_cube_max[1], small_cube_min[2]], [small_cube_min[0], small_cube_min[1], small_cube_min[2]]],
    [small_cube_min, [small_cube_max[0], small_cube_min[1], small_cube_min[2]], [small_cube_max[0], small_cube_max[1], small_cube_min[2]], [small_cube_min[0], small_cube_max[1], small_cube_min[2]]],
    [small_cube_max, [small_cube_max[0], small_cube_min[1], small_cube_max[2]], [small_cube_min[0], small_cube_min[1], small_cube_max[2]], [small_cube_min[0], small_cube_max[1], small_cube_max[2]]]
]
small_cube = Poly3DCollection(verts_small, alpha=0.2, facecolor='red')
ax.add_collection3d(small_cube)


# 设置图形的范围
ax.set_xlim(0, 50)
ax.set_ylim(0, 50)
ax.set_zlim(0, 50)
ax.set_aspect('equal', adjustable='box')

# 添加标题和标签
# plt.title('A3 Map')
# plt.xlabel('X Position')
# plt.ylabel('Y Position')
# 显示自定义图例
ax.legend(loc='best', bbox_to_anchor=(1, 0.5), fontsize=10, prop={'family': 'Times New Roman', 'size': 10})

# 调整坐标轴朝内
ax.tick_params(direction='in', labelsize=10)

# 调整坐标轴标签字体大小和样式
ax.set_xticklabels(ax.get_xticks(), fontsize=10, fontname='Times New Roman')
ax.set_yticklabels(ax.get_yticks(), fontsize=10, fontname='Times New Roman')
ax.set_zticklabels(ax.get_zticks(), fontsize=10, fontname='Times New Roman')
ax.view_init(elev=16, azim=-135)
# 显示图形
plt.savefig('mapA3.pdf', dpi=600, bbox_inches='tight')
plt.show()

