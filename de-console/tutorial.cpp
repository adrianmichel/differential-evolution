/*
 * Copyright (c) 2017, Adrian Michel
 * http://www.amichel.com
 *
 * This software is released under the 3-Clause BSD License
 *
 * The complete terms can be found in the attached LICENSE file
 * or at https://opensource.org/licenses/BSD-3-Clause
 */

#include <differential_evolution.hpp>
#include <iostream>

using namespace amichel::de;

/**
 * Objective function to optimize is the "sphere function":
 *
 * f(x,y) = x^2 + y^2
 */
double sphere_function(amichel::de::DVectorPtr args) {
  /**
    * The two function arguments are the elements index 0 and 1 in
    * the argument vector, as defined by the constraints vector
    * below
    */
  double x = (*args)[0];
  double y = (*args)[1];

  return x * x + y * y;
}

#define VARS_COUNT 20
#define POPULATION_SIZE 200

void simpleUsage() {
  try {
    /**
     * Create and initialize the constraints object
     *
     * First create it with default constraints (double type, min
     * -1.0e6, max 1.0e6) then set the first two elements to be of
     *  type real with x between -10, 10 and y between -100, 100.
     */
    constraints_ptr constraints(
      std::make_shared<constraints>(VARS_COUNT, -1.0e6, 1.0e6));
    (*constraints)[0] = std::make_shared<real_constraint>(-10, 10);
    (*constraints)[1] = std::make_shared<real_constraint>(-100, 100);

    /**
     * Instantiate two null listeners, one for the differential
     * evolution, the other one for the processors
     */
    listener_ptr listener(std::make_shared<null_listener>());
    processor_listener_ptr processor_listener(
      std::make_shared<null_processor_listener>());

    /**
     * Instantiate the collection of processors with the number of
     * parallel processors (4), the objective function and the
     * listener
     */
    processors::processors_ptr _processors(
      std::make_shared<processors >(4, boost::ref(sphere_function),
        processor_listener));

    /**
     * Instantiate a simple termination strategy which will stop the
     * optimization process after 10000 generations
     */
    termination_strategy terminationStrategy(max_gen_termination_strategy(10000));

    /**
     * Instantiate the selection strategy - we'll use the best of
     * parent/child strategy
     */
    selection_strategy selectionStrategy{best_parent_child_selection_strategy{}};

    /**
     * Instantiate the mutation strategy - we'll use the mutation
     * strategy 1 with the weight and crossover factors set to 0.5
     * and 0.9 respectively
     */
    mutation_strategy_arguments mutation_arguments(0.5, 0.9);
    mutation_strategy_ptr mutationStrategy(
        std::make_shared<mutation_strategy_1>(VARS_COUNT,
                                                mutation_arguments));

    /**
     * Instantiate the differential evolution using the previously
     * defined constraints, processors, listener, and the various
     * strategies
     */
    differential_evolution de(
        VARS_COUNT, POPULATION_SIZE, _processors, constraints, true,
        terminationStrategy, selectionStrategy, mutationStrategy, listener);

    /**
     * Run the optimization process
     */
    de.run();

    /**
     * Get the best individual resulted from the optimization
     * process
     */
    individual_ptr best(de.best());

    /**
     * Print out the result
     */
    std::cout << "minimum value for the " << /*of->name() << */ " is "
              << best->cost() << " for x=" << (*best->vars())[0]
              << ", y=" << (*best->vars())[1];
  } catch (const amichel::de::exception& e) {
    /**
     * Print out any errors that happened during the initialization
     * or optimization phases
     */
    std::cout << "an error occurred: " << e.what();
  }
}
