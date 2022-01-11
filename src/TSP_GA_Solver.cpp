#include <cstdint>
#include <vector>
#include <iostream>
#include <tuple>
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
static constexpr bool PRINT_FLAG = true; // true to print during Genetic Algorithm

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

    // run GA Solver to get the result, std::vector<int> and float
    auto [index_route_vec, cost] = Run(num_nodes, population_size, max_iter, distance_matrix, PRINT_FLAG);

    // index_route_vec is a vector of route node index, 0 means the agent, 1 ~ n means the fitst ~ n-th target
    // example: index_route_vec = [0, 4, 3, 1, 2, 5], 0 means the agent, 1 ~ 5 means the fitst ~ 5-th target,
    // the task allocation order is agent 0 -> T4 -> T3 -> T1 -> T2 -> T5

    // need to map index route to path index, if we have route x -> y, we want to map [x,y] to the corresponding index of path_many, 
    // which indicates the route x -> y.

    // start_goal_pair is the start-goal index pair, path_many stores paths along entries of start_goal_pair
    // For example: there is 1 agent and 5 targers, start_goal_pair (row-major) = 
    //    [0,1,  0,2,  0,3,  0,4,  0,5,
    //     1,2,  1,3,  1,4,  1,5,
    //     2,3,  2,4,  2,5,
    //     3,4,  3,5,
    //     4,5]

    // Assume Route [x,y], x < y, is located in x-row, (y-x-1)-column, so there are (x-1) rows before [x,y], denote N = num_targets.
    // So the total number of elements before x-row is N + (N-1) + (N-2) + ... + (N-(x-1)) = (( N + (N-(x-1)) ) * x) / 2 = ( (2*N-x+1)*x ) / 2
    // This formula also holds when x = 0.
    // And the number of elements in x-row before [x,y] is y - x - 1, note indices start from 0.
    // So the index of path_many corresponding to route [x,y] is (2*N-x+1)*x/2 + y-x-1
    // size_t index_x_to_y = (2*N-x+1)*x/2 + y-x-1;
    // std::vector<int> path_x_to_y = path_many[index_x_to_y];

    // When Route [x,y], x > y, locate the path index for route [y,x], then the path can be obtained by reversing the path of route [y,x].

    int x, y; // the route index [x,y]
    size_t idx_path; // the corresponding path index for route [x,y]
    std::vector<std::vector<int>> path_many_result; // the paths given route [x,y]
    for (size_t i = 0; i < index_route_vec.size()-1; ++i) {
        if (index_route_vec[i] < index_route_vec[i+1]) {
            x = index_route_vec[i];
            y = index_route_vec[i+1];
            idx_path = static_cast<size_t>( (2*(num_nodes-1)-x+1)*x/2 + y-x-1 );
            path_many_result.push_back(path_many[idx_path]);
        }
        else {
            x = index_route_vec[i+1];
            y = index_route_vec[i];
            idx_path = static_cast<size_t>( (2*(num_nodes-1)-x+1)*x/2 + y-x-1 );
            std::vector<int> path_now = path_many[idx_path];

            // if route [x,y], x > y, we need to reverse path_now
            // let path_now = [x0,y0, x1,y1, x2,y2], then path_reversed = [x2,y2, x1,y1, x0,y0]
            // so it's NOT an exact vector reverse

            // swap the first entry with the second to last, swap the second with the last
            // swap the third with the forth to last, swap the forth with the third to last
            // until met the central 2 entries, skip them

            // when the path is empty, do not swap
            for (int i = 0; i < static_cast<int>(path_now.size()/2) - 1; i = i + 2) {
                std::iter_swap(path_now.begin()+i, path_now.end()-i-2);
                std::iter_swap(path_now.begin()+i+1, path_now.end()-i-1);
            }
            path_many_result.push_back(path_now);
        }
    }

    // target_idx_order = [0, 3, 4, 2, 1] means the task allocation order is
    // T0 -> T3 -> T4 -> T2 -> T1, where T0 is the first task
    std::vector<size_t> target_idx_order;
    for (size_t i = 0; i < index_route_vec.size()-1; ++i) {
        target_idx_order.push_back(index_route_vec[i+1] - 1);
    }

    return {path_many_result, target_idx_order, cost};
}


inline PYBIND11_MODULE(TSP_GA_Solver, module) {
    module.doc() = "Python wrapper of TSP_GA_Solver";

    module.def("Solve", &Solve, "Solve the traveling salesperson problem by GA algorithm");

}
