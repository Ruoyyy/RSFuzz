<<<<<<< HEAD
# RSFuzz prototype for Pietro

## Environmental Requirements

### MATLAB environment
- MATLAB R2019b or later
- Required Toolboxes:
  - Mapping Toolbox
  - Statistics and Machine Learning Toolbox
  - Optimization Toolbox

### Python Environment
- Python 3.10
- Anaconda
- Required Python Libraries:
  - numpy
  - matplotlib
  - scipy

## Dependency Installation

### 1. Python environment configuration
```bash
# Creating a Python Virtual Environment
conda create -n drone-fuzzing python=3.10
conda activate drone-fuzzing

# Install required libraries
pip install numpy matplotlib scipy
```

### Python Path Configuration

**Important**: You need to modify the Python interpreter and module paths in the following files:

- `SA_Fuzzing.m` (line 4)
- `MA_Fuzzing.m` (line 4)

```matlab
% Change to your Python interpreter path
pyenv('Version','Your Python Path\\python.exe');

% Change to your Python module path (line 5)
python_module_path = 'Your Project Path\\';
```

## Project Structure

```
Pietro/
├── Configuration Files
│   ├── easy_simulation_config.m     # Simple Scenario Configuration
│   ├── medium_simulation_config.m   # Medium Scenario Configuration
│   └── hard_simulation_config.m     # Hard Scenario Configuration
│
├── Core Algorithms
│   ├── advanced_quadrotor_swarm.m   # Main Swarming Algorithm
│   ├── advanced_agent_control.m     # Advanced Agent Control
│   ├── medium_quadrotor_swarm.m     # Medium Complexity Swarming Algorithm
│   └── agent_control3.m             # Basic Agent Control
│
├── Fuzzing
│   ├── SA_Fuzzing.m                 
│   └── MA_Fuzzing.m                 
│
├── Python Modules
│   ├── attack.py                    # Attack Strategy Implementation
│   ├── robustness.py               # Robustness Evaluation
│   └── katz.py                      # Graph Centrality Algorithm
│
├── Map Data
│   ├── Map/                         # Map Visualization Files
│   ├── *_obstacles.mat              # Obstacle Data
│   └── *_targets.mat                # Target Data
│
└── Map Generation
    ├── map_maker.m                  # Map Generator
    └── map_maker_FIX.m             # Fixed Map Generator
```

## How to Run

### 1. Basic Swarm Algorithm Runing
```matlab
% Run in the MATLAB command window
run('hard_simulation_config.m');  % Load the configuration
advanced_quadrotor_swarm;          % Run the swarm algorithm
```

### 2. SA-Fuzzing
```matlab
% Ensure the Python environment is correctly configured
run('hard_simulation_config.m');  % Load the configuration
SA_Fuzzing;                        % Run SA-Fuzzing
```

### 3. MA-Fuzzing
```matlab
% Ensure the Python environment is correctly configured
run('hard_simulation_config.m);  % Load the configuration
MA_Fuzzing;                        % Run MA-Fuzzing
```

### 4. Testing Scenarios of Different Difficulties
```matlab
% Easy scenario
run('easy_simulation_config.m');
% Medium difficulty scenario
run('medium_simulation_config.m');
% Hard scenario
run('hard_simulation_config.m');
```

## Configuration Instructions

### Main configuration parameters

The following parameters can be adjusted in `easy/medium/hard_simulation_config.m`:

```matlab
% Basic simulation parameters
N = 15; % Number of drones
t_end = 10000; % Simulation end time (seconds)
dt = 5e-1; % Simulation time step (seconds)

% Drone physical parameters
r_agent = 0.5; % Agent detection/communication radius (meters)
IR_dist = 0.1; % Infrared sensor range (meters)
v_max = 0.05; % Maximum speed (meters/second)
agent_radius = 0.1; % Agent radius (meters)

% Mission parameters
target_radius = 0.1; % Target radius (meters)
R_search = 0.9; % Target search rate (/second)
map_size = 5; % Map size (meters)
```
=======
# RSFuzz prototype for Pietro

## Environmental Requirements

### MATLAB environment
- MATLAB R2019b or later
- Required Toolboxes:
  - Mapping Toolbox
  - Statistics and Machine Learning Toolbox
  - Optimization Toolbox

### Python Environment
- Python 3.10
- Anaconda
- Required Python Libraries:
  - numpy
  - matplotlib
  - scipy

## Dependency Installation

### 1. Python environment configuration
```bash
# Creating a Python Virtual Environment
conda create -n drone-fuzzing python=3.10
conda activate drone-fuzzing

# Install required libraries
pip install numpy matplotlib scipy
```

### Python Path Configuration

**Important**: You need to modify the Python interpreter and module paths in the following files:

- `SA_Fuzzing.m` (line 4)
- `MA_Fuzzing.m` (line 4)

```matlab
% Change to your Python interpreter path
pyenv('Version','Your Python Path\\python.exe');

% Change to your Python module path (line 5)
python_module_path = 'Your Project Path\\';
```

## Project Structure

```
Pietro/
├── Configuration Files
│   ├── easy_simulation_config.m     # Simple Scenario Configuration
│   ├── medium_simulation_config.m   # Medium Scenario Configuration
│   └── hard_simulation_config.m     # Hard Scenario Configuration
│
├── Core Algorithms
│   ├── advanced_quadrotor_swarm.m   # Main Swarming Algorithm
│   ├── advanced_agent_control.m     # Advanced Agent Control
│   ├── medium_quadrotor_swarm.m     # Medium Complexity Swarming Algorithm
│   └── agent_control3.m             # Basic Agent Control
│
├── Fuzzing
│   ├── SA_Fuzzing.m                 
│   └── MA_Fuzzing.m                 
│
├── Python Modules
│   ├── attack.py                    # Attack Strategy Implementation
│   ├── robustness.py               # Robustness Evaluation
│   └── katz.py                      # Graph Centrality Algorithm
│
├── Map Data
│   ├── Map/                         # Map Visualization Files
│   ├── *_obstacles.mat              # Obstacle Data
│   └── *_targets.mat                # Target Data
│
└── Map Generation
    ├── map_maker.m                  # Map Generator
    └── map_maker_FIX.m             # Fixed Map Generator
```

## How to Run

### 1. Basic Swarm Algorithm Runing
```matlab
% Run in the MATLAB command window
run('hard_simulation_config.m');  % Load the configuration
advanced_quadrotor_swarm;          % Run the swarm algorithm
```

### 2. SA-Fuzzing
```matlab
% Ensure the Python environment is correctly configured
run('hard_simulation_config.m');  % Load the configuration
SA_Fuzzing;                        % Run SA-Fuzzing
```

### 3. MA-Fuzzing
```matlab
% Ensure the Python environment is correctly configured
run('hard_simulation_config.m);  % Load the configuration
MA_Fuzzing;                        % Run MA-Fuzzing
```

### 4. Testing Scenarios of Different Difficulties
```matlab
% Easy scenario
run('easy_simulation_config.m');
% Medium difficulty scenario
run('medium_simulation_config.m');
% Hard scenario
run('hard_simulation_config.m');
```

## Configuration Instructions

### Main configuration parameters

The following parameters can be adjusted in `easy/medium/hard_simulation_config.m`:

```matlab
% Basic simulation parameters
N = 15; % Number of drones
t_end = 10000; % Simulation end time (seconds)
dt = 5e-1; % Simulation time step (seconds)

% Drone physical parameters
r_agent = 0.5; % Agent detection/communication radius (meters)
IR_dist = 0.1; % Infrared sensor range (meters)
v_max = 0.05; % Maximum speed (meters/second)
agent_radius = 0.1; % Agent radius (meters)

% Mission parameters
target_radius = 0.1; % Target radius (meters)
R_search = 0.9; % Target search rate (/second)
map_size = 5; % Map size (meters)
```
>>>>>>> 30ef24b (How to run)
