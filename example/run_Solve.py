#!/usr/bin/env python3
import time
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    from Simulator import Simulator
    import TSP_GA_Solver


if __name__ == "__main__":
    # define the world
    map_width_meter = 25.0
    map_height_meter = 25.0
    map_resolution = 2
    value_non_obs = 0 # the cell is empty
    value_obs = 255 # the cell is blocked
    # create a simulator
    MySimulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)
    # number of obstacles
    num_obs = 250
    # [width, length] size of each obstacle [meter]
    size_obs = [0.5, 0.5]
    # generate random obstacles
    MySimulator.generate_random_obs(num_obs, size_obs)
    # convert 2D numpy array to 1D list
    world_map = MySimulator.map_array.flatten().tolist()

    # This is for an agent and some targets
    num_targets = 10
    agent_position, targets_position = MySimulator.generate_start_and_goals(num_targets)

    # some hyper-parameters for Genetic Algorithm
    population_size = 50
    max_iter = 100
    
    # solve it
    t0 = time.time()
    path_many_result, task_allocation_order, cost = TSP_GA_Solver.Solve(agent_position, targets_position,
                        population_size, max_iter,
                        world_map, MySimulator.map_width, MySimulator.map_height)
    t1 = time.time()
    print("Time used for a single path is [sec]: " + str(t1-t0))
    print("Cost: ", cost)
    print("Task allocation order: ", task_allocation_order)

    # visualization
    MySimulator.plot_many_path(path_many_result, agent_position, targets_position)
    plt.show()
