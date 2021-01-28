/*
Adapted from https://www.geeksforgeeks.org/traveling-salesman-problem-using-genetic-algorithm/
I made some changes to eliminate some inefficient loops.
*/

#ifndef TSP_GA_SOLVER_HPP
#define TSP_GA_SOLVER_HPP


#include <limits.h>
#include <ctime>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <random>
#include <typeinfo>


// Structure of a GNOME 
// string defines the path traversed by the salesman, and the fitness value of the path is stored in an integer 
struct individual {
	std::string gnome;
	int fitness;
};


struct settings_solver {
	// Initial population size for the algorithm 
	unsigned int pop_size;
	// Number of cities in TSP
	unsigned int num_nodes;
	// Number of Gene Iterations 
	unsigned int gen_thres;
};


// generate a random integer number in [start, end]
inline int rand_num_simple(const int& start, const int& end)
{
	int r = end - start;
	int rnum;

	if (start != end) {
		rnum = start + (rand() % r) + 1;
	}
	else {
		rnum = start;
	}

	return rnum;
}


// generate two unrepeated random integers without the while loop
inline void rand_num(const std::vector<int>& idx_mutate_vector, int* rand_num_array)
{
	std::vector<int> idx_mutate_vector_copy = idx_mutate_vector;

	// generate the first random number
	rand_num_array[0] = rand_num_simple(1, idx_mutate_vector.size());
	// erase the first random number.
	// if the random mumber is 4, the index of the original vector is 4-1=3.
	idx_mutate_vector_copy.erase(idx_mutate_vector_copy.begin() + rand_num_array[0] - 1);

	// generate a random index for the new vector
	//int index_2 = rand_num_simple(0, idx_mutate_vector_copy.size() - 1);

	rand_num_array[1] = idx_mutate_vector_copy[rand_num_simple(0, idx_mutate_vector_copy.size() - 1)];

	// return rand_num_array_ptr
}


// Function to return a mutated GNOME 
// Mutated GNOME is a string with a random interchange of two genes to create variation in species 
inline std::string mutatedGene(std::string& gnome, const std::vector<int>& idx_mutate_vector)
{
	int rand_num_array[2];
	rand_num(idx_mutate_vector, rand_num_array);
	std::swap(gnome[rand_num_array[0]], gnome[rand_num_array[1]]);

	return gnome;
}


// Function to return a valid GNOME string 
// required to create the population 
inline std::string create_gnome(const std::vector<int>& idx_mutate_vector)
{
	std::string gnome = "0";
	std::vector<int> idx_mutate_vector_copy = idx_mutate_vector;

	while (idx_mutate_vector_copy.size() >= 1) {
		// generate a random index
		int idx_now = rand_num_simple(0, idx_mutate_vector_copy.size() - 1);
		// slice the vector, convert the integer to string
		gnome += (char)(idx_mutate_vector_copy[idx_now] + 48);
		// erase the used one
		idx_mutate_vector_copy.erase(idx_mutate_vector_copy.begin() + idx_now);
	}

	// end with 0, the starting point
	gnome += gnome[0];

	return gnome;
}


// Function to return the fitness value of a gnome. 
// The fitness value is the path length of the path represented by the GNOME. 
inline int cal_fitness(const std::string& gnome, int **map_pointer)
{
	int f = 0;
	for (unsigned int i = 0; i < gnome.size() - 1; i++) {
		//std::cout << typeid(gnome[i] - 48).name() << std::endl; // int
		//std::cout << typeid(gnome[i]).name() << std::endl; //char

		if (map_pointer[gnome[i] - 48][gnome[i + 1] - 48] == INT_MAX)
			return INT_MAX;
		f += map_pointer[gnome[i] - 48][gnome[i + 1] - 48];
	}
	return f;
}


// Function to return the updated value of the cooling element. 
inline int cooldown(const int& temp)
{
    return (90 * temp) / 100;
}


// Comparator for GNOME struct. 
inline bool lessthan(const struct individual& t1, const struct individual& t2)
{
	return t1.fitness < t2.fitness;
}


// Utility function for TSP problem. 
inline void TSP_Solve(int **map_pointer, const struct settings_solver& settings)
{
	// Generation Number 
	unsigned int gen = 1;

	/*
	!! Suppose we have 5(num_nodes) cities, the chromosome has 6 numbers, because we want to come back.
	But the first and last element won't mutate, so the mutateable numbers are num_nodes+1-2=num_nodes-1=4.
	In this case, the vector is [1, 2, 3, 4]
	*/
	std::vector<int> idx_mutate_vector(settings.num_nodes-1);
	std::iota(idx_mutate_vector.begin(), idx_mutate_vector.end(), 1);

	std::vector<struct individual> population;
	struct individual temp;

	// Populating the GNOME pool. 
	for (unsigned int i = 0; i < settings.pop_size; i++) {
		temp.gnome = create_gnome(idx_mutate_vector);
		temp.fitness = cal_fitness(temp.gnome, map_pointer);
		population.push_back(temp);
	}

	std::cout << "\nInitial population: " << std::endl
		<< "GNOME     FITNESS VALUE\n";
	for (unsigned int i = 0; i < settings.pop_size; i++)
		std::cout << population[i].gnome << "   "
		<< population[i].fitness << std::endl;
	std::cout << "\n";

	int temperature = 10000;

	// Iteration to perform 
	// population crossing and gene mutation. 
	while (temperature > 1000 && gen <= settings.gen_thres) {
		sort(population.begin(), population.end(), lessthan);
		std::cout << "\nCurrent temp: " << temperature << "\n";
		std::vector<struct individual> new_population;

		for (unsigned int i = 0; i < settings.pop_size; i++) {
			struct individual p1 = population[i];

			while (true) {
				//std::string new_g = mutatedGene(p1.gnome, settings.num_nodes);
				std::string new_g = mutatedGene(p1.gnome, idx_mutate_vector);


				struct individual new_gnome;
				new_gnome.gnome = new_g;
				new_gnome.fitness = cal_fitness(new_gnome.gnome, map_pointer);

				if (new_gnome.fitness <= population[i].fitness) {
					new_population.push_back(new_gnome);
					break;
				}
				else {

					// Accepting the rejected children at 
					// a possible probablity above threshold. 
					double prob = pow(2.7,
						-1 * ((double)(new_gnome.fitness
							- population[i].fitness)
							/ temperature));
					if (prob > 0.5) {
						new_population.push_back(new_gnome);
						break;
					}
				}
			}
		}

		temperature = cooldown(temperature);
		population = new_population;
		std::cout << "Generation " << gen << " \n";
		std::cout << "GNOME     FITNESS VALUE\n";

		for (unsigned int i = 0; i < settings.pop_size; i++)
			std::cout << population[i].gnome << "   "
			<< population[i].fitness << std::endl;
		gen++;
	}
}

#endif