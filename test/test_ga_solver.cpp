/*
Adapted from https://www.geeksforgeeks.org/traveling-salesman-problem-using-genetic-algorithm/
I made some changes to eliminate some inefficient loops.
*/

#include <limits>
#include <ctime>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <random>
#include <typeinfo>
#include "ga_solver.hpp"


int main()
{
    std::clock_t time_start;
    double time_duration;

    time_start = std::clock();

    // Number of nodes in TSP
    const size_t num_nodes = 5;
    // Initial population size for the algorithm 
    const size_t pop_size = 50;
    // Number of Gene Iterations 
    const size_t gen_thres = 30;
    
    std::vector<std::vector<float>> distance_matrix{
        { 0, 2, 100, 12, 5 },
        { 2, 0, 4, 8, 20 },
        { 3, 4, 0, 3, 3 },
        { 12, 8, 3, 0, 10 },
        { 5, 1, 3, 10, 0 } };

    // std::vector<std::vector<float>> distance_matrix{
    //     { 0, 1, 6, 3, 7 },
    //     { 1, 0, 2, 10, 4 },
    //     { 6, 2, 0, 2, LARGE_NUM },
    //     { 3, 10, 2, 0, 13 },
    //     { 7, 4, LARGE_NUM, 13, 0 } };


    Run(num_nodes, pop_size, gen_thres, distance_matrix);

    time_duration = (std::clock() - time_start) / (double)CLOCKS_PER_SEC;
    std::cout << "Time used [sec]: " << time_duration << std::endl;
}