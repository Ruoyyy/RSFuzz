# RSFuzz prototype for Howard's

## Environmental Requirements
- MATLAB R2020b or newer (Python interface enabled)
- Python 3.8+ (match your MATLAB-supported version if possible)
- OS: Windows 10/11 (other platforms may require path tweaks)

### Dependency Installation
1) Python packages
```bash
pip install networkx matplotlib numpy
```

2) Bind MATLAB to Python (choose one)
- Set Python interpreter in MATLAB (recommended):
```matlab
% One-time setup: point to your python.exe
pyenv('Version', 'C:\\Path\\to\\Python\\python.exe');
```
- Or keep MATLAB's default `pyenv`. At runtime, `fuzzer.m` / `MA_fuzzer.m` will add `src/` to `py.sys.path` so that `katz.py` can be imported.

3) Python path (auto-configured)
- The code runs the following at runtime:
```matlab
this_dir = fileparts(mfilename('fullpath'));
if count(py.sys.path, this_dir) == 0
    insert(py.sys.path, int32(0), this_dir);
end
```

## Project Structure
```
Howard/
├─ README.md
├─ src/
│  ├─ SA_Fuzzing.m            # SA-style fuzzing entry (subset graph)
│  ├─ MA_Fuzzing.m            # MA advanced fuzzing entry (global graph)
│  ├─ fuzzer.m                # SA attack point selection
│  ├─ MA_fuzzer.m             # MA attack point selection
│  ├─ Robustness.m            # robustness metrics and aggregation
│  ├─ GradientDescentUpdate.m # position update
│  ├─ Drone.m
│  ├─ Obstacle.m
│  ├─ Waypoints.m
│  ├─ Monitor.m
│  ├─ katz.py                 # 3D graph + Katz centrality
│  └─ Map/
│     ├─ A3_map.png
│     ├─ A3map.pdf
│     ├─ map3.png
│     └─ map.py
└─ Result/                    # output logs (auto-created)
```

## How to Run
In MATLAB, switch current folder to the project root (or `addpath('src')`).

1) Basic Swarm Algorithm Running (no attack)

```matlab
run('Test_Swarm_3d.m')
```

2) SA-Fuzzing (Simulated-annealing-like; graph on influenced subset S)
```matlab
run('src/SA_Fuzzing.m')
```
Outputs:
- Console: prints per-iteration `[x, y, z, robustness]`
- File: `Result/iteration_log_<Nd>.txt` (Nd = number of swarm drones, excluding the attack drone), one line per iteration `[x, y, z, robustness]`

3) MA-Fuzzing (Multi-Agent advanced; global graph)
```matlab
run('src/MA_Fuzzing.m')
```
Outputs are the same as SA (same Nd-based log filename). If you prefer separate files, rename to `iteration_log_MA_<Nd>.txt`.

## Configuration Instructions
- Swarm size and radius:
  - Adjust `Nd` and `radius` in `SA_Fuzzing.m` / `MA_Fuzzing.m`.
- Targets/waypoints:
  - Edit `end_loc` and `pt1/pt2/pt3` in the two scripts.
- Attack drone init:
  - Randomly initialized around a swarm drone; see `DEL_attack` and `attackdrone = Drone(5, ..., 3)`.
- Katz edge threshold:
  - SA: in `fuzzer.m`, `max_influence_distance = 1.5 * drones(S(1)).radius;`
  - MA: in `MA_fuzzer.m`, `max_influence_distance = 1.5 * drones(1).radius;`
- Influenced subset (SA):
  - Criterion `d <= AttackDrone.radius + drone.radius`.
- Candidate points:
  - `num_points` defaults to 20; change in `fuzzer.m` / `MA_fuzzer.m`.
- Logging:
  - Both scripts write `./Result/iteration_log_<Nd>.txt` and print the same line to console.

## Notes
- If `networkx` or `matplotlib` is missing or Python is unavailable, the code falls back to a safe branch without centrality filtering. Installing the dependencies is recommended for intended behavior.
A3 will be released after the paper review process.