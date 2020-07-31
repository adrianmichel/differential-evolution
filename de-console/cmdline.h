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
 * \defgroup de-console de-console command line application
 * @{
 */

#pragma once

#include <de_constraints.hpp>
#include <mutation_strategy.hpp>
#include <selection_strategy.hpp>
#include <processors.hpp>

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
  static amichel::de::ObjectiveFunction _functions[];
  size_t m_functionToOptimize;
  amichel::de::selection_strategy _selectionStrategy;
  size_t _processorsCount;
  size_t _argumentsCount;
  double m_argumentsDefConstraintMin;
  double m_argumentsDefConstraintMax;
  std::vector<std::string> m_constraints;

 public:
  CmdLine();
  bool process(int argc, char* argv[]);

  std::string getUsage() const;

  void showUsage();
  void notice(std::ostream& os);
  void showError(const std::string& error);

  size_t populationSize() const noexcept { return _populationSize; }
  size_t maxGenerations() const noexcept { return _maxGenerations; }
  amichel::de::mutation_strategy_ptr mutationStrategy() const noexcept {
    return _mutationStrategy;
  }
  size_t processorsCount() const noexcept { return _processorsCount; }
  double weight() const noexcept { return _weight; }
  double crossover() const noexcept { return _crossover; }
  bool minimize() const noexcept { return _minimize; }
  amichel::de::constraints constraints() const noexcept { 
    return amichel::de::constraints(m_constraints, _argumentsCount, m_argumentsDefConstraintMin, m_argumentsDefConstraintMax);
  }
  size_t argumentsCount() const noexcept { return _argumentsCount; }
  amichel::de::ObjectiveFunction functionToOptimize() const {
    return _functions[m_functionToOptimize];
  }
  const amichel::de::selection_strategy& selectionStrategy() const noexcept {
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
