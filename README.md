# 🏢 3D Multi-Robot Rescue Simulation

A real-time simulation of autonomous rescue robots navigating a 3D building to locate and rescue survivors, optimized using a **Genetic Algorithm (GA)** with **A* pathfinding**.

## 🎯 Overview

This project simulates a disaster rescue scenario where multiple robots must:
- Navigate a 3D grid-based building with obstacles
- Detect survivors using simulated heat and CO2 sensors
- Compute optimal rescue paths while minimizing risk and travel distance
- Coordinate to avoid redundant assignments

The **Genetic Algorithm** evolves rescue plans over generations, optimizing both:
1. **Survivor Assignment** — Which robot rescues which survivors
2. **Visit Sequence** — The order each robot visits its assigned survivors

## 🧬 Genetic Algorithm

| Component | Implementation |
|-----------|----------------|
| **Chromosome** | Robot positions + ordered survivor sequences |
| **Selection** | Tournament selection (size 3) |
| **Crossover** | Single-point crossover on robot assignments |
| **Mutation** | Swap, Remove, Transfer, Reverse operations |
| **Fitness** | Maximizes rescues, minimizes path length & risk |
| **Elitism** | Top 10% preserved each generation |

## ⚡ Parallel Processing (IPC)

Fitness evaluation is parallelized using **POSIX IPC**:

- **Shared Memory** (`shm_open` + `mmap`) — Zero-copy data sharing between processes
- **Semaphores** — Work distribution and synchronization
- **Process Pool** (`fork`) — Worker processes compute fitness in parallel

## 🗺️ Pathfinding

- **A\* Algorithm** for optimal 3D pathfinding
- **6-directional movement** (±X, ±Y, ±Z)
- **Risk-aware** path cost calculation
- Handles obstacles and blocked cells

## 🖥️ Visualization

Real-time **OpenGL/GLUT** 3D visualization showing:
- Building structure with obstacles
- Survivor locations
- Robot paths and movements
- Risk heatmap

## 📁 Project Structure
~~~
├── include/           # Header files (.h)
├── src/               # Source files (.c)
│   ├── main.c         # Entry point
│   ├── ga.c           # Genetic Algorithm
│   ├── ga_parallel.c  # Parallel processing (IPC)
│   ├── astar.c        # A* Pathfinding
│   ├── grid.c         # 3D Grid management
│   ├── config.c       # Config parser
│   └── visualize.c    # OpenGL visualization
├── build/             # Object files
├── configfile.txt     # Parameters
└── Makefile           # Build script
~~~
## ⚙️ Configuration

Edit `configfile.txt` to customize:
~~~
GRID_X = 20              # Building dimensions
GRID_Y = 20
GRID_Z = 20
OBSTACLE_DENSITY = 0.3   # 30% obstacles
ROBOT_COUNT = 8          # Number of rescue robots
POPULATION_SIZE = 100    # GA population
GENERATIONS = 200        # Evolution iterations
MUTATION_RATE = 0.3      # 30% mutation chance
POOL_SIZE = 4            # Parallel worker processes
~~~
## Dependencies
- GCC compiler
- OpenGL & GLUT libraries
- POSIX-compliant system (Linux/WSL)

## 📊 Output

The program compares:
1. **Optimal A\* Solution** — Exhaustive/greedy baseline
2. **GA Solution** — Evolved rescue plan

## 🎓 Key Concepts Demonstrated
* Genetic Algorithms for combinatorial optimization
* A* pathfinding in 3D space
* Inter-Process Communication (IPC) with shared memory
* Process synchronization with semaphores
* Real-time 3D visualization with OpenGL
