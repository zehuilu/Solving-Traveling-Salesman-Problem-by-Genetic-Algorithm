/*
Adapted from https://www.geeksforgeeks.org/traveling-salesman-problem-using-genetic-algorithm/
I made some changes to eliminate some inefficient loops.
*/

#ifndef GA_SOLVER_HPP
#define GA_SOLVER_HPP


#include <limits>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <random>
#include <typeinfo>


static constexpr float LARGE_NUM = 1E8;

std::random_device rd;     // only used once to initialise (seed) engine
std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
std::uniform_real_distribution<float> dist_01(0.0, 1.0);  // uniform real distribution [0.0, 1.0]


// Structure of a GNOME 
// string defines the path traversed by the salesman, and the fitness value of the path is stored in an integer 
struct individual {
    std::string gnome;
    int fitness;
};

// generate a random integer number in [min, max]
inline int rand_integer(const int& min, const int& max)
{
    std::uniform_int_distribution<int> uni(min, max); // guaranteed unbiased
    return uni(rng);
}

// generate two unrepeated random integers without the while loop
inline void rand_integer_two(const std::vector<int>& idx_mutate_vector, int* rand_num_array)
{
    std::vector<int> idx_mutate_vector_copy(idx_mutate_vector);

    // generate the first random number
    rand_num_array[0] = rand_integer(1, idx_mutate_vector.size());
    // erase the first random number.
    // if the random mumber is 4, the index of the original vector is 4-1=3.
    idx_mutate_vector_copy.erase(idx_mutate_vector_copy.begin() + rand_num_array[0] - 1);

    // generate a random index for the new vector
    //int index_2 = rand_integer(0, idx_mutate_vector_copy.size() - 1);

    rand_num_array[1] = idx_mutate_vector_copy[rand_integer(0, idx_mutate_vector_copy.size()-1)];

    // return rand_num_array_ptr
}

// Function to return a mutated GNOME 
// Mutated GNOME is a string with a random interchange of two genes to create variation in species 
inline std::string mutatedGene(std::string& gnome, const std::vector<int>& idx_mutate_vector)
{
    int rand_num_array[2];
    rand_integer_two(idx_mutate_vector, rand_num_array);
    std::swap(gnome[rand_num_array[0]], gnome[rand_num_array[1]]);

    return gnome;
}

// Function to return a valid GNOME string 
// required to create the population 
inline std::string create_gnome(const std::vector<int>& idx_mutate_vector)
{
    std::string gnome = "0";
    std::vector<int> idx_mutate_vector_copy(idx_mutate_vector);

    while (idx_mutate_vector_copy.size() >= 1) {
        // generate a random index
        int idx_now = rand_integer(0, idx_mutate_vector_copy.size() - 1);
        // slice the vector, convert the integer to string
        gnome += (char)(idx_mutate_vector_copy[idx_now] + 48);
        // erase the used one
        idx_mutate_vector_copy.erase(idx_mutate_vector_copy.begin() + idx_now);
    }

    // comment this because don't need to come back
    // // end with 0, the starting point
    // gnome += gnome[0];

    return gnome;
}

// Function to return the fitness value of a gnome. 
// The fitness value is the path length of the path represented by the GNOME. 
inline float cal_fitness(const std::string& gnome, const std::vector<std::vector<float>>& distance_matrix)
{
    float f = 0;
    for (unsigned int i = 0; i < gnome.size() - 1; i++) {
        //std::cout << typeid(gnome[i] - 48).name() << std::endl; // int
        //std::cout << typeid(gnome[i]).name() << std::endl; //char

        if (distance_matrix[gnome[i] - 48][gnome[i + 1] - 48] >= LARGE_NUM)
            return LARGE_NUM;
        f += distance_matrix[gnome[i] - 48][gnome[i + 1] - 48];
    }
    return f;
}

// Function to return the updated value of the cooling element. 
inline float cooldown(const float& temp)
{
    return 0.9 * temp;
    // return temp - 10;
}

// Comparator for GNOME struct. 
inline bool lessthan(const struct individual& t1, const struct individual& t2)
{
    return t1.fitness < t2.fitness;
}

// Utility function for TSP problem. 
inline void Run(
    const size_t& num_nodes,
    const size_t& pop_size,
    const size_t& max_iter,
    const std::vector<std::vector<float>>& distance_matrix)
{
    // Generation Number 
    unsigned int iter = 1;

    /*
    !! Suppose we have 5(num_nodes) cities, the chromosome has 5 numbers, because we don't want to come back.
    But the first element won't mutate, so the mutateable numbers are num_nodes-1 = 4.
    In this case, the vector is [1, 2, 3, 4]
    */
    std::vector<int> idx_mutate_vector(num_nodes-1);
    std::iota(idx_mutate_vector.begin(), idx_mutate_vector.end(), 1);

    std::vector<struct individual> population;
    struct individual temp;

    // Populating the GNOME pool. 
    for (unsigned int i = 0; i < pop_size; i++) {
        temp.gnome = create_gnome(idx_mutate_vector);
        temp.fitness = cal_fitness(temp.gnome, distance_matrix);
        population.push_back(temp);
    }

    sort(population.begin(), population.end(), lessthan);
    std::cout << "\nInitial population: " << std::endl << "GNOME     FITNESS VALUE\n";
    for (unsigned int i = 0; i < pop_size; i++)
        std::cout << population[i].gnome << "   "
        << population[i].fitness << std::endl;
    std::cout << "\n";

    float temperature = 100;

    // Iteration to perform 
    // population crossing and gene mutation. 
    while (temperature > 5 && iter <= max_iter) {
        std::cout << "\nCurrent temp: " << temperature << "\n";

        std::vector<struct individual> new_population;

        for (unsigned int i = 0; i < pop_size; i++) {
            struct individual p1 = population[i];

            while (true) {
                //std::string new_g = mutatedGene(p1.gnome, num_nodes);
                std::string new_g = mutatedGene(p1.gnome, idx_mutate_vector);


                struct individual new_gnome;
                new_gnome.gnome = new_g;
                new_gnome.fitness = cal_fitness(new_gnome.gnome, distance_matrix);

                if (new_gnome.fitness <= population[i].fitness) {
                    new_population.push_back(new_gnome);
                    break;
                }
                else {
                    float random_prob = dist_01(rng);

                    // Accepting the rejected children at 
                    // a possible probablity above threshold. 
                    float prob = pow(2.71828, -1*(new_gnome.fitness-population[i].fitness)/temperature);
                    if (random_prob < prob) {
                        new_population.push_back(new_gnome);
                        break;
                    }
                }
            }
        }

        temperature = cooldown(temperature);
        population = new_population;
        sort(population.begin(), population.end(), lessthan);

        std::cout << "Generation " << iter << " \n";
        std::cout << "GNOME     FITNESS VALUE\n";
        for (unsigned int i = 0; i < pop_size; i++)
            std::cout << population[i].gnome << "   "
            << population[i].fitness << std::endl;

        iter++;
    }
}

#endif