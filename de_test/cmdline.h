/*
 * Copyright (c) 2017, Adrian Michel
 * http://www.amichel.com
 *
 * This software is released under the 3-Clause BSD License
 *
 * The complete terms can be found in the attached LICENSE file
 * or at https://opensource.org/licenses/BSD-3-Clause
 */

/**
 * \defgroup de_test de_test command line application
 * @{
 */

#ifndef DE_TEST_CMDLINE_H_INCLUDED
#define DE_TEST_CMDLINE_H_INCLUDED

// MS compatible compilers support #pragma once

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include <de_constraints.hpp>
#include <mutation_strategy.hpp>
#include <selection_strategy.hpp>

#include "objective_function.h"

/**
 * Encapsulation of the function to optimize
 */
class FunctionToOptimize {
  static objective_function_ptr _functions[];
  size_t _index;

 public:
  FunctionToOptimize(size_t index);

  objective_function_ptr operator()() const;
};

/**
 * Shared pointer to a FunctionToOptimize
 */
using FunctionToOptimizePtr = std::shared_ptr<FunctionToOptimize>;

/**
 * Command line class
 */
class CmdLine {
 private:
  std::string _description;
  double _weight;
  double _crossover;
  size_t _populationSize;
  size_t _maxGenerations;
  amichel::de::mutation_strategy_ptr _mutationStrategy;
  bool _minimize;
  FunctionToOptimizePtr _functionToOptimize;
  amichel::de::selection_strategy_ptr _selectionStrategy;
  size_t _processorsCount;
  amichel::de::constraints_ptr _constraints;
  size_t _argumentsCount;

 public:
  CmdLine();
  bool process(int argc, char* argv[]);

  std::string getUsage() const;

  void showUsage();
  void notice(std::ostream& os);
  void showError(const std::string& error);

  size_t populationSize() const { return _populationSize; }
  size_t maxGenerations() const { return _maxGenerations; }
  amichel::de::mutation_strategy_ptr mutationStrategy() const {
    return _mutationStrategy;
  }
  size_t processorsCount() const { return _processorsCount; }
  double weight() const { return _weight; }
  double crossover() const { return _crossover; }
  bool minimize() const { return _minimize; }
  amichel::de::constraints_ptr constraints() const { return _constraints; }
  size_t argumentsCount() const { return _argumentsCount; }
  objective_function_ptr functionToOptimize() const {
    assert(_functionToOptimize);
    return (*_functionToOptimize)();
  }
  amichel::de::selection_strategy_ptr selectionStrategy() const {
    assert(_selectionStrategy);
    return _selectionStrategy;
  }
};

/**
 * Command line exception
 */
class CmdLineException : public amichel::de::exception {
 public:
  CmdLineException(const std::string& message)
      : amichel::de::exception(message.c_str()) {}
};

/**
 * @}
 */

#endif  // DE_TEST_CMDLINE_H_INCLUDED
