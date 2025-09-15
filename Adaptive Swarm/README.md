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

## Project Structure

```
Adaptive Swarm/
├─ .python-version
├─ README.md
├─ requirements.txt    #Top-level instructions and dependency list.
├─ spec.txt
├─ log_file/
├─ test/
│  ├─ monitor.py
│  ├─ rtamt_discrete_time_test.py
│  └─ rtamt_test.py
├─ tool/    #A collection of tool scripts.
│  ├─ __pycache__/
│  ├─ AssertionExtraction.py    #Script for extracting assertions from specifications/logs.
│  ├─ log_tool.py
│  ├─ monitor.py
│  ├─ test.py
│  └─ tool.py
└─ adaptive_swarm/
   ├─ CITATION.cff
   ├─ CMakeLists.txt
   ├─ LICENSE
   ├─ MSc_Thesis_Skoltech.pdf    #Full text of the relevant master's thesis.
   ├─ README.md
   ├─ package.xml
   ├─ dataset/    #Dataset placeholders or data file directories.
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
   └─ scripts/    #Main algorithms and example scripts, such as hierarchical path planning, social force models, random walks, and human potential fields; swarmlib.py is a public library.
      ├─ gradient_interactive/...
      ├─ human_potential_fields/...
      ├─ layered_planner/...
      ├─ random_walk/...
      ├─ social_force_model/...
      ├─ GradientBasedPlanning.ipynb
      └─ swarmlib.py
```

## How to run

### Basic

```bash
# Enter the project root directory
cd "RSFuzz\Adaptive Swarm"
```

### Running the Example

```bash
# Run fuzz.py (located in the adaptive_swarm/scripts/layered_planner directory)
python adaptive_swarm/scripts/layered_planner/fuzz.py
# or
cd adaptive_swarm/scripts/layered_planner
python fuzz.py

# Run a hierarchical planner simulation
python adaptive_swarm/scripts/layered_planner/layered_planner_sim.py

# Run the standard hierarchical planner
python adaptive_swarm/scripts/layered_planner/layered_planner.py
```

## Configuration Instructions

### Main parameters (in the Params class of fuzz.py)
- `animate_rrt`: Whether to display the RRT construction process (1 to show, 0 to hide to save time)
- `visualize`: Whether to display the robot motion visualization (1 to show, 0 to hide)
- `postprocessing`: Whether to process and visualize simulation experiment data (1 to enable, 0 to disable)
- `savedata`: Whether to save post-processing metrics to an XLS file (1 to save, 0 to not save)
- `maxiters`: Maximum number of samples to construct the RRT (default 500)
- `goal_prob`: Probability of sampling the target point (default 0.05)

### Basic parameters

- `influence_radius`: Influence radius (default: 0.15m)
- `interrobots_dist`: Robot distance (default: 0.3m)
- `num_robots`: Number of robots (default: 4)
- `drone_vel`: Drone speed (default: 4.0 m/s)

### Test Parameters

Fuzzing parameters can be adjusted by modifying the `Params` class in the code:

```python
class Params:
    def __init__(self):
        self.num_robots = 4
        self.interrobots_dist = 0.3
        self.influence_radius = 0.15
        self.drone_vel = 4.0
        # Other parameters...
```

## Different Fuzzing Modes

```python
# The main function of fuzz.py
if __name__ == '__main__':
    for _ in range(20):
        attack_demo = attack(attack_strategy=1)
        # 1:Random-Fuzzing;2:SA-Fuzzing;3:MA-Fuzzing
        attack_demo.start()
```

### Debug Mode

Enable verbose logging:

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```
