import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - needed for 3D projection

class Graph:
    def __init__(self, drone_list, max_influence_distance):
        self.nodes = []
        self.edges = {}
        self.max_influence_distance = max_influence_distance  # 三维下的最大影响距离（欧氏距离）
        self.add_nodes(drone_list)
        self.analyze_graph()  # 计算脆弱节点

    def add_nodes(self, drone_list):
        # drone_list: iterable of (x, y, z)
        for drone_id, coords in enumerate(drone_list):
            if len(coords) != 3:
                raise ValueError("Graph requires 3D coordinates: (x, y, z)")
            x, y, z = coords
            self.nodes.append((drone_id, float(x), float(y), float(z)))
            self.edges[drone_id] = []

    def calculate_edges(self):
        # 三维欧氏距离阈值内连边
        for i, (node_id1, x1, y1, z1) in enumerate(self.nodes):
            for j, (node_id2, x2, y2, z2) in enumerate(self.nodes):
                if i != j:
                    dx = x1 - x2
                    dy = y1 - y2
                    dz = z1 - z2
                    distance = (dx * dx + dy * dy + dz * dz) ** 0.5
                    if distance <= self.max_influence_distance:
                        self.edges[node_id1].append((node_id2, distance))

    def get_edges(self, node_id):
        return self.edges[node_id]

    def katz_centrality(self):
        # 将Graph对象转换为networkx图（中心性与维度无关，只依赖图结构）
        G = nx.Graph()
        for node_id, x, y, z in self.nodes:
            G.add_node(node_id, pos=(x, y, z))
        for node_id in self.edges:
            for neighbor, distance in self.edges[node_id]:
                G.add_edge(node_id, neighbor, weight=distance)

        katz_centrality = nx.katz_centrality(G, alpha=0.1, beta=1.0, max_iter=10000, tol=0.1)
        return katz_centrality

    def find_vulnerable_nodes(self, katz_centrality):
        # 找出Katz中心性值最大的节点
        max_centrality = max(katz_centrality.values())
        vulnerable_nodes = [node_id for node_id, centrality in katz_centrality.items() if centrality == max_centrality]
        return vulnerable_nodes

    def analyze_graph(self):
        # 计算图中的边
        self.calculate_edges()

        # 计算Katz中心性
        self.katz_centrality_values = self.katz_centrality()

        # 找出关键节点
        self.vulnerable_nodes = self.find_vulnerable_nodes(self.katz_centrality_values)

def visualize_graph(graph):
    # 3D 可视化
    G = nx.Graph()
    for node_id, x, y, z in graph.nodes:
        G.add_node(node_id, pos=(x, y, z))
    for node_id in graph.edges:
        for neighbor, distance in graph.edges[node_id]:
            G.add_edge(node_id, neighbor, weight=distance)

    pos = nx.get_node_attributes(G, 'pos')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 节点绘制
    xs = [pos[n][0] for n in G.nodes()]
    ys = [pos[n][1] for n in G.nodes()]
    zs = [pos[n][2] for n in G.nodes()]
    colors = ['red' if n in graph.vulnerable_nodes else 'lightblue' for n in G.nodes()]
    ax.scatter(xs, ys, zs, c=colors, s=80)

    # 标注节点编号
    for n in G.nodes():
        x, y, z = pos[n]
        ax.text(x, y, z, str(n), fontsize=9)

    # 边绘制
    for u, v, data in G.edges(data=True):
        x1, y1, z1 = pos[u]
        x2, y2, z2 = pos[v]
        ax.plot([x1, x2], [y1, y2], [z1, z2], color='gray', linewidth=1)

    plt.show()
# # 生成随机整数点
# num_points = 10
# max_coordinate = 10
# points_set = set()
# drone_list = []

# while len(drone_list) < num_points:
#     x = np.random.randint(0, max_coordinate)
#     y = np.random.randint(0, max_coordinate)
#     point = (x, y)
#     if point not in points_set:
#         points_set.add(point)
#         drone_list.append(point)
# drone_list = [(1,1), (2,2), (3,3)]

# max_influence_distance = 5  # 设置最大影响距离

# # 创建一个图对象并传入无人机列表和最大影响距离
# graph = Graph(drone_list, max_influence_distance)

# # 打印每个节点的Katz中心性
# for node_id, centrality in graph.katz_centrality_values.items():
#     print(f"drone {node_id} Katz centrality: {centrality}")

# # 可视化图
# visualize_graph(graph)
