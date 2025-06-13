import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

class Graph:
    def __init__(self, drone_list, max_influence_distance):
        self.nodes = []
        self.edges = {}
        self.max_influence_distance = max_influence_distance  # 新增参数：最大影响距离
        self.add_nodes(drone_list)
        self.analyze_graph()  # 计算脆弱节点

    def add_nodes(self, drone_list):
        for drone_id, (x, y) in enumerate(drone_list):            
            x_rounded = round(x, 2)
            y_rounded = round(y, 2)
            self.nodes.append((drone_id, x, y))
            self.edges[drone_id] = []

    def calculate_edges(self):
        for i, (node_id1, x1, y1) in enumerate(self.nodes):
            for j, (node_id2, x2, y2) in enumerate(self.nodes):
                if i != j:
                    distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
                    if distance <= self.max_influence_distance:  # 新增条件：距离小于等于最大影响距离
                        self.edges[node_id1].append((node_id2, distance))

    def get_edges(self, node_id):
        return self.edges[node_id]

    def katz_centrality(self):
        # 将Graph对象转换为networkx图
        G = nx.Graph()
        for node_id, x, y in self.nodes:
            G.add_node(node_id, pos=(x, y))
        for node_id in self.edges:
            for neighbor, distance in self.edges[node_id]:
                G.add_edge(node_id, neighbor, weight=distance)

        # 使用networkx计算Katz中心性
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
    G = nx.Graph()
    for node_id, x, y in graph.nodes:
        G.add_node(node_id, pos=(x, y))
    for node_id in graph.edges:
        for neighbor, distance in graph.edges[node_id]:
            G.add_edge(node_id, neighbor, weight=distance)

    pos = nx.get_node_attributes(G, 'pos')
    node_colors = ['red' if node_id in graph.vulnerable_nodes else 'lightblue' for node_id in G.nodes()]
    nx.draw(G, pos, with_labels=True, node_color=node_colors, node_size=500, font_size=10, font_weight='bold')

    for edge in G.edges(data=True):
        x1, y1 = pos[edge[0]]
        x2, y2 = pos[edge[1]]
        distance = edge[2]['weight']
        plt.annotate(f"{distance:.2f}", xy=((x1 + x2) / 2, (y1 + y2) / 2), xytext=(-10, 10), textcoords='offset points', fontsize=8)

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
