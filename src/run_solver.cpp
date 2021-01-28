/*
Adapted from https://www.geeksforgeeks.org/traveling-salesman-problem-using-genetic-algorithm/
I made some changes to eliminate some inefficient loops.
*/

#include <limits.h>
#include <ctime>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <random>
#include <typeinfo>
#include "tsp_ga_solver.hpp"


int main()
{
	std::clock_t time_start;
	double time_duration;

	time_start = std::clock();

	struct settings_solver settings;
	const unsigned int num_nodes = 5;
	settings.num_nodes = num_nodes;
	settings.pop_size = 10;
	settings.gen_thres = 10;
	
	int map[num_nodes][num_nodes] = { { 0, 2, INT_MAX, 12, 5 },
				  { 2, 0, 4, 8, INT_MAX },
				  { INT_MAX, 4, 0, 3, 3 },
				  { 12, 8, 3, 0, 10 },
				  { 5, INT_MAX, 3, 10, 0 } };


	/*int map[num_nodes][num_nodes] = { { 0, 1, 6, 3, 7 },
					  { 1, 0, 2, 10, 4 },
					  { 6, 2, 0, 2, INT_MAX },
					  { 3, 10, 2, 0, 13 },
					  { 7, 4, INT_MAX, 13, 0 } };
					  */

	// number of rows
	int *map_pointer[num_nodes];
	for (unsigned int x = 0; x < settings.num_nodes; ++x) map_pointer[x] = map[x];

	TSP_Solve(map_pointer, settings);

	time_duration = (std::clock() - time_start) / (double)CLOCKS_PER_SEC;

	std::cout << "Time used [sec]: " << time_duration << std::endl;
}