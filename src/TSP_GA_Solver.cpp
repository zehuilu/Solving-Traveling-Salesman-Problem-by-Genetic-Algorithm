#include <cstdint>
#include <vector>

#include <iostream>
#include <tuple>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <math.h>
#include <limits>
#include <algorithm>
#include <future>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


#include "tileadaptor.hpp"
#include "utility.hpp"
#include "get_combination.hpp"
#include "ga_solver.hpp"


static constexpr float WEIGHT_PATH = 1E2; // weight for path finder


/*
SolveOneAgent: Find all the collision-free paths from every element to another element in start+targets,
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
inline std::tuple< std::vector<std::vector<int>>, std::vector<size_t> > Solve(
    std::vector<int> &agent_position,
    std::vector<int> &targets_position,
    const size_t &population_size,
    const size_t &max_iter,
    const std::vector<int> &Map,
    const int &mapSizeX,
    const int &mapSizeY)
{
    int num_nodes = targets_position.size()/2 + 1; // number of targets + one agent
    std::vector<int> start_goal_pair = get_combination(num_nodes, 2);
    std::vector<std::vector<int>> path_many;
    std::vector<std::vector<float>> distance_matrix(num_nodes, std::vector<float>(num_nodes, 0));
    int start[2];
    int goal[2];

    // Instantiating our path adaptor
    // passing the map size, and the map
    Vectori mapSize(mapSizeX, mapSizeY);
    TileAdaptor adaptor(mapSize, Map);
    // This is a bit of an exageration here for the weight, but it did make my performance test go from 8s to 2s
    Pathfinder pathfinder(adaptor, WEIGHT_PATH);

    for (size_t idx = 0; idx < start_goal_pair.size(); idx = idx + 2) {
        int start_idx = start_goal_pair[idx];
        int goal_idx = start_goal_pair[idx+1];

        if (start_idx != 0) {
            start[0] = targets_position[2*(start_idx-1)];
            start[1] = targets_position[2*(start_idx-1)+1];
        }
        else {
            start[0] = agent_position[0];
            start[1] = agent_position[1];
        }

        if (goal_idx != 0) {
            goal[0] = targets_position[2*(goal_idx-1)];
            goal[1] = targets_position[2*(goal_idx-1)+1];

        }
        else {
            goal[0] = agent_position[0];
            goal[1] = agent_position[1];
        }

        // doing the search, when infeasible, Path is empty, Distance = 0
        // when start = goal, Path = start and Distance is empty
        // std::vector<int>, float
        auto [Path, Distance] = pathfinder.search(start[1]*mapSizeX+start[0], goal[1]*mapSizeX+goal[0], mapSize);

        // assign distance to distance matrix
        int i = num_nodes - 2 - floor(sqrt(-8*(idx/2) + 4*num_nodes*(num_nodes-1)-7)/2.0 - 0.5);
        int j = (idx/2) + i + 1 - (num_nodes*(num_nodes-1))/2 + ((num_nodes-i)*((num_nodes-i)-1))/2;

        if (Path.size() > 2) {
            distance_matrix[i][j] = Distance;
            distance_matrix[j][i] = distance_matrix[i][j];
        }
        else if (Path.size() == 2) {
            // when start = goal, let Path = {start, goal}, and Distance = 0
            Path.push_back(Path[0]);
            Path.push_back(Path[1]);
            distance_matrix[i][j] = 0.0;
            distance_matrix[j][i] = distance_matrix[i][j];
        }
        else {
            // if no feasible, set as a large number, but not the maximum of int64_t
            distance_matrix[i][j] = LARGE_NUM;
            distance_matrix[j][i] = distance_matrix[i][j];
        }

        path_many.push_back(Path);
        
        // Regenerate the neighbors for next run
        // if (idx < start_goal_pair.size()-1)
        pathfinder.generateNodes();
    }

    // std::cout << "This is distance_matrix: " << std::endl;
    // for (size_t i = 0; i < distance_matrix.size(); i++)
    // {
    //     for (size_t j = 0; j < distance_matrix[i].size(); j++)
    //     {
    //         std::cout << distance_matrix[i][j] << ", ";
    //     }
    //     std::cout << std::endl;
    // }





    // run GA Solver to get the result
    Run(num_nodes, population_size, max_iter, distance_matrix);





}


inline PYBIND11_MODULE(TSP_GA_Solver, module) {
    module.doc() = "Python wrapper of TSP_GA_Solver";

    module.def("Solve", &Solve, "Solve the traveling salesperson problem by GA algorithm");

}
