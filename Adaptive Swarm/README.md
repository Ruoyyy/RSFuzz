# RSFuzz prototype for Adaptive Swarm

## Environmental requirements

- Python 3.7+
- OS：Windows/Linux/macOS

## Dependency Installation

### 1. Install Python dependencies

```bash
pip install -r requirements.txt
```

### 2. Description of core dependency libraries

- **numpy**: Numerical calculations and array operations
- **matplotlib**: Data visualization and plotting
- **scipy**: Scientific Computing Libraries
- **networkx**: Graph theory and network analysis
- **svgpath2mpl**: SVG path parsing
- **psutil**: System resource monitoring
- **rtamt**: STL (Signal Temporal Logic) specification library

## 项目结构

```
Adaptive Swarm/
├─ .python-version
├─ README.md
├─ requirements.txt     #Top-level instructions and dependency list.
├─ spec.txt
├─ log_file/
├─ test/
│  ├─ monitor.py
│  ├─ rtamt_discrete_time_test.py
│  └─ rtamt_test.py
├─ tool/
│  ├─ __pycache__/
│  ├─ AssertionExtraction.py
│  ├─ log_tool.py
│  ├─ monitor.py
│  ├─ test.py
│  └─ tool.py
└─ adaptive_swarm/
   ├─ CITATION.cff
   ├─ CMakeLists.txt
   ├─ LICENSE
   ├─ MSc_Thesis_Skoltech.pdf
   ├─ README.md
   ├─ package.xml
   ├─ dataset/
   ├─ figures/
   │  ├─ critically_damped/
   │  ├─ formation4/...
   │  ├─ formation6/...
   │  ├─ formation8/...
   │  ├─ imp_modeles.png
   │  ├─ layered_planner/
   │  │  ├─ narrow_passage/...
   │  │  ├─ navigation.png
   │  │  ├─ repulsive_potential1D.png
   │  │  ├─ rr_path.png
   │  │  └─ surface_potential_trajs.png
   │  ├─ oscillations/
   │  ├─ overdamped/...
   │  ├─ passage_3_drones.gif
   │  ├─ passage_3_drones_payload.gif
   │  ├─ social_force_model/...
   │  └─ underdamped/...
   ├─ launch/
   │  ├─ connect123.launch
   │  └─ layered_planner.launch
   └─ scripts/
      ├─ gradient_interactive/...
      ├─ human_potential_fields/...
      ├─ layered_planner/...
      ├─ random_walk/...
      ├─ social_force_model/...
      ├─ GradientBasedPlanning.ipynb
      └─ swarmlib.py
```



## 运行方法

### 基本运行

```bash
# 进入项目根目录
cd "RSFuzz\Adaptive Swarm"

# 运行fuzz.py（位于adaptive_swarm/scripts/layered_planner目录）
python adaptive_swarm/scripts/layered_planner/fuzz.py
```

### 或者直接进入脚本目录运行

```bash
cd adaptive_swarm/scripts/layered_planner
python fuzz.py
```

### 运行示例

```bash
# 从项目根目录运行
python adaptive_swarm/scripts/layered_planner/fuzz.py

# 或进入脚本目录后运行
cd adaptive_swarm/scripts/layered_planner
python fuzz.py

# 运行分层规划器仿真
python adaptive_swarm/scripts/layered_planner/layered_planner_sim.py

# 运行标准分层规划器
python adaptive_swarm/scripts/layered_planner/layered_planner.py
```

## 配置说明

### 主要参数（在fuzz.py的Params类中）
- `animate_rrt`: 是否显示RRT构建过程（1显示，0隐藏以减少时间）
- `visualize`: 是否显示机器人运动可视化（1显示，0隐藏）
- `postprocessing`: 是否处理和可视化仿真实验数据（1启用，0禁用）
- `savedata`: 是否保存后处理指标到XLS文件（1保存，0不保存）
- `maxiters`: 构建RRT的最大采样次数（默认500）
- `goal_prob`: 采样目标点的概率（默认0.05）

### 系统监控
- 使用Monitor类进行鲁棒性计算和STL规范监控
- 支持CPU和内存使用率监控
- 集成势场算法和RRT路径规划

### 基础参数

- `influence_radius`: 影响半径 (默认: 0.15m)
- `interrobots_dist`: 机器人间距 (默认: 0.3m)
- `num_robots`: 机器人数量 (默认: 4)
- `drone_vel`: 无人机速度 (默认: 4.0 m/s)

### 测试参数

可以通过修改代码中的`Params`类来调整测试参数：

```python
class Params:
    def __init__(self):
        self.num_robots = 4
        self.interrobots_dist = 0.3
        self.influence_radius = 0.15
        self.drone_vel = 4.0
        # 其他参数...
```

## 输出结果

运行后会生成以下输出：

- 实时可视化窗口显示无人机群体运动
- 控制台输出测试日志和鲁棒性指标
- 可选的数据文件保存测试结果

## 故障排除

### 常见问题

1. **ImportError**: 确保所有依赖都已正确安装
2. **matplotlib显示问题**: 在Linux上可能需要安装GUI后端
   ```bash
   sudo apt-get install python3-tk
   ```
3. **权限问题**: 确保有写入输出目录的权限

### 调试模式

启用详细日志输出：

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 贡献

欢迎提交Issue和Pull Request来改进本项目。

## 许可证

本项目采用MIT许可证，详见LICENSE文件。