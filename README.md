# Solving-Traveling-Salesman-Problem-by-Genetic-Algorithm
This repository solves Traveling Salesman Problem by Genetic Algorithm. This solver is adapted from https://www.geeksforgeeks.org/traveling-salesman-problem-using-genetic-algorithm/. I made some changes to eliminate some inefficient loops.


To-Do:
1. Compile it into a library such that you can use it in Python.
2. Compare results with [OR-Tools](https://developers.google.com/optimization).


This repo has been tested with:
* GCC 9.3.0, CMake 3.16.3, Ubuntu 20.04.1 LTS
* Clang 12.0.0.0, CMake 3.18.3, macOS 10.15.7


The solver is `include/tsp_ga_solver.hpp`. And an example is `src/run_solver.hpp`.


Build
=====

```
$ cd <MAIN_DIRECTORY>
$ mkdir build
$ cd build
$ cmake ..
$ make
```


Run
===
```
$ cd <MAIN_DIRECTORY>/build
$ ./run_solver
```
