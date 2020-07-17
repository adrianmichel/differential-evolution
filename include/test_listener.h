#pragma once
#include <listener.hpp>
#include <individual.hpp>
#include <iostream>

/**
 * Basic Differential Evolution listener - displays the
 * generation cout and the best cost.
 */
class DETestListener : public amichel::de::listener {
public:
  virtual void start() {}

  virtual void end() {}

  virtual void error() {}

  virtual void startGeneration(size_t genCount) {}

  virtual void endGeneration(size_t genCount, amichel::de::individual_ptr bestIndGen,
    amichel::de::individual_ptr bestInd) {
    std::cout << (boost::format("genCount: %1%, cost: %2%\n") % genCount %
      bestInd->cost())
      .str();
  }

  virtual void startSelection(size_t genCount) {}

  virtual void endSelection(size_t genCount) {}

  virtual void startProcessors(size_t genCount) {}

  virtual void endProcessors(size_t genCount) {}
};
