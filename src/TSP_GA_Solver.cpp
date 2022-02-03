#include <vector>
#include <tuple>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "SolveSingle.hpp"


/*
Solve: Find all the collision-free paths from every element to another element in start+targets,
    and return an optimal order for the agent to explore all the targets, and the concatenated path given the order.

Input:
    agent_position: 1D integer array [x, y] for the agent position
    targets_position: 1D integer array [x0,y0, x1,y1, x2,y2, ...] for the targets positions
    Map: 1D integer array for the map, flattened by a 2D array map; 0 for no obstacles, 255 for obstacles
    mapSizeX: integer for the width of the map
    mapSizeY: integer for the height of the map

Output:
    path_many_result: 2D integer array for all the index paths given the task allocation order,
        [[x0,y0, ..., x1,y1], [x1,y1, ..., x2,y2], [x2,y2, ..., x3,y3], ...]
    target_idx_order: 1D integer array for task allocation order, 
        [0, 4, 3, 1, 2, 5] means the task allocation order is T0 -> T4 -> T3 -> T1 -> T2 -> T5,
        where T0 is the first task.
*/
inline std::tuple< std::vector<std::vector<int>>, std::vector<size_t>, float > Solve(
    const std::vector<int> &agent_position,
    const std::vector<int> &targets_position,
    const size_t &population_size,
    const size_t &max_iter,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    return SolveSingle(agent_position, targets_position, population_size, max_iter, Map, mapSizeX, mapSizeY);
}


inline PYBIND11_MODULE(TSP_GA_Solver, module) {
    module.doc() = "Python wrapper of TSP_GA_Solver";

    module.def("Solve", &Solve, "Solve the traveling salesperson problem by GA algorithm");
}
