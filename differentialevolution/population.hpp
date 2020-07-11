/*
 * Copyright (c) 2017, Adrian Michel
 * http://www.amichel.com
 *
 * This software is released under the 3-Clause BSD License
 *
 * The complete terms can be found in the attached LICENSE file
 * or at https://opensource.org/licenses/BSD-3-Clause
 */

#pragma once

#include "individual.hpp"

namespace amichel {
namespace de {

typedef std::vector<individual_ptr> population_base;

/**
 * A collection of individuals.
 */
class population : public population_base {
 public:
  /**
   * constructs a population object containing uninitialized
   * individuals
   *
   * @param popSize the population size (number of individuals)
   * @param varCount the number of variables for each individual
   */
  population(size_t popSize, size_t varCount) : population_base(popSize) {
    assert(popSize > 0);
    assert(varCount > 0);

    init(popSize, varCount);
  }

  /**
   * constructs a population object and initializes its
   * individuals by first setting all variables to random values
   * within the limits imposed by the corresponding constraints,
   * then runs the objective function for each individual to
   * calculate the associated cost.
   *
   * @param popSize the population size (number of individuals)
   * @param varCount the number of variables for each individual
   * @param constraints constraints to use when initializing the
   *  				  individuals
   */
  population(size_t popSize, size_t varCount, constraints_ptr constraints)
      : population_base(popSize) {
    assert(popSize > 0);
    assert(varCount > 0);

    init(popSize, varCount);

    for (population::size_type i = 0; i < size(); ++i) at(i)->init(constraints);
  }

  /**
   * returns the best individual in a population
   *
   * @return individual_ptr
   */
  individual_ptr best(bool minimize) const {
    population::size_type best(0);

    for (population::size_type i = 0; i < size(); ++i)
      best = at(i)->better_or_equal(at(best), minimize) ? i : best;

    return at(best);
  }

  /**
   * returns the string representation of a population
   *
   * @return std::string string representation of the population
   */
  std::string to_string() {
    std::string str;
    for (population::size_type i = 0; i < size(); ++i)
      str += at(i)->to_string();

    return str;
  }

 private:
  void init(size_t popSize, size_t varCount) {
    for (population_base::size_type i = 0; i < size(); ++i)
      operator[](i) = boost::make_shared<individual>(varCount);
  }

 public:
};

typedef boost::shared_ptr<population> population_ptr;

}  // namespace de
}  // namespace amichel