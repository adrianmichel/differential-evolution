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

#include <functional>

namespace amichel {
namespace de {

/**
 * Abstract base class defining the interface used by a concrete
 * Termination Strategy class
 *
 * A termination strategy defines the logic used to stop the
 * optimization process. Can be as simple as a generation
 * counter that exits when the max number of generations has
 * been reached, or can implement more complex algorithms that
 * try to minimize the number of execution steps by
 * determinining when a reasonable best value has been reached.
 */
  /**
   * @param best The best individual so far
   * @param genCount generation number
   *
   * @return bool return true to continue, or false to stop the
   *  	   optimization process
   */

using termination_strategy = std::function<bool(individual_ptr, size_t)>;

/**
 * A smart pointer to a TerminationStrategy
 */
using termination_strategy_ptr = std::shared_ptr<termination_strategy>;

/**
 * Basic implementation of a Termination Strategy: stop the
 * optimization process if a maximum number of generations has
 * been reached
 */
class max_gen_termination_strategy{
 private:
  const size_t m_maxGen;

 public:
  /**
   * constructs a max_gen_termination_strategy object
   *
   * @param maxGen maximum number of generations after which the
   *  			 optimization stops
   */
  max_gen_termination_strategy(size_t maxGen) : m_maxGen(maxGen) {}

  bool operator()(individual_ptr best, size_t genCount) {
    return genCount < m_maxGen;
  }
};

}  // namespace de
}  // namespace amichel