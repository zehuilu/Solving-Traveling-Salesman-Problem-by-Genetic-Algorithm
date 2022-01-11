# Solving-Traveling-Salesman-Problem-by-Genetic-Algorithm
This repository solves Single-Agent Task Allocation and Path Finding Problem (SA-TA-PF) by modeling it as a Traveling Salesperson Problem (TSP) with Genetic Algorithm. The solver, **TSP_GA_Solver**, is written in C++ and is compiled as a Python library. You can either use it by C++ or Python.

For TSP, the agent needs to go back to its original position. But for SA-TA-PF, the agent does not have to. If you want to solve an exact TSP, it is easy to do that by editing my codes in `include/ga_solver.hpp` and `TSP_GA_Solver.cpp`. Basically, you need to append `0` to the end of the chromosome `std::vector<int> gnome`, and keep the last element of this chromosome as `0` during the mutation `mutatedGene()`.


This repo has been tested with:
* GCC 9.3.0, CMake 3.16.3, Ubuntu 20.04.1 LTS
* Clang 13.0.0.0, CMake 3.22.1, macOS 11.4
* Clang 12.0.0.0, CMake 3.18.3, macOS 10.15.7


Dependencies
============
* [Lazy-Theta-with-optimization-any-angle-pathfinding](https://github.com/zehuilu/Lazy-Theta-with-optimization-any-angle-pathfinding)
  - [pybind11](https://github.com/pybind/pybind11)
  - [numpy](https://numpy.org/)
  - [matplotlib](https://matplotlib.org/)


Build
=====

Since [Lazy-Theta-with-optimization-any-angle-pathfinding](https://github.com/zehuilu/Lazy-Theta-with-optimization-any-angle-pathfinding) is a submodule of this repo, follow the instructions below to build [Lazy-Theta-with-optimization-any-angle-pathfinding](https://github.com/zehuilu/Lazy-Theta-with-optimization-any-angle-pathfinding) and this repo respectively.


To download this repo and build [Lazy-Theta-with-optimization-any-angle-pathfinding](https://github.com/zehuilu/Lazy-Theta-with-optimization-any-angle-pathfinding),
```
$ sudo apt install build-essential # For macOS: xcode-select --install && brew install cmake
$ apt install python3-pybind11 # For macOS: brew install pybind11
$ pip3 install numpy matplotlib
$ git clone https://github.com/zehuilu/Solving-Traveling-Salesman-Problem-by-Genetic-Algorithm
$ cd <MAIN_DIRECTORY>
$ git submodule update --init --recursive
$ cd <MAIN_DIRECTORY>/externals/Lazy-Theta-with-optimization-any-angle-pathfinding
$ mkdir build
$ cd build
$ cmake ..
$ make
```


To build this repo,
```
$ cd <MAIN_DIRECTORY>
$ mkdir build
$ cd build
$ cmake ..
$ make
```


Usage
=====


* To run the solver in Python:
```
$ cd <MAIN_DIRECTORY>
$ python3 example/run_Solve.py
```

* To run the solver in C++, see how I use it in `test/test_ga_solver.py`.


Example
=======

**Python**

To call TSP_GA_Solver in Python, a simple example is shown below. More details are in `example/run_Solve.py`.

```python
import TSP_GA_Solver
map_width = 25
map_height = 25
# world_map is a 1D list (row-major), 0 means no obstacles, 255 means blocked by obstacles
agent_position = [5, 8]  # [px, py] for the agent
targets_position = [35,34, 20,25, 31,40]  # [px0,py0, px1,py1, px2,py2] for the targets
# solve it
paths, task_order, cost = TSP_GA_Solver.Solve(agent_position, targets_position,
                                              population_size, max_iter,
                                              world_map, MySimulator.map_width, MySimulator.map_height)
```

Run `example/run_Solve.py`, the result is shown below. Time used is 0.261 sec.
Note: The obstacle map is generated randomly.


![Example](doc/example.png?raw=true "Example")


To-Do by Jan. 11, 2022
======================
1. Revise and add some comments in my codes

