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

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>

#include <de_constraints.hpp>
#include <mutation_strategy.hpp>
#include <selection_strategy.hpp>

#include "objective_function.h"

/**
 * Encapsulation of the function to optimize
 *
 * @author adrian (12/8/2011)
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
typedef std::shared_ptr<FunctionToOptimize> FunctionToOptimizePtr;

/**
 * Command line class
 *
 * @author adrian (12/8/2011)
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
 *
 * @author adrian (12/8/2011)
 */
class CmdLineException : public amichel::de::exception {
 public:
  CmdLineException(const std::string& message)
      : amichel::de::exception(message.c_str()) {}
};

/**
 * @}
 */

namespace amichel {
namespace de {
/** 
 * Collection of constraints. Support initialization from strings.
 */
class constraints_str : public constraints {
 private:
  typedef boost::char_separator<char> separator;
  typedef boost::tokenizer<separator> tokenizer;

 public:
  constraints_str(size_t varCount, double defMin, double defMax)
      : constraints(varCount, defMin, defMax) {}

  /**
   * Initializes a collection of constraints from string
   * descsriptions. Currently used only for range based
   * constraints.
   *
   * A constraint can be described as "type;min;max" where type
   * can be real or integer and min and max are the range limits.
   *
   * @author adrian (12/1/2011)
   *
   * @param str a a collection (vector) of constraint description
   *        strings, each string describing constraints for
   *        one variable
   * @param var_count the total number of variables (can be
   *           different than the number of strings). If
   *           there are more variables than strings, the
   *           extra constraints are set to be real and use
   *           the default min and max arguments for the
   *           range
   * @param def_min default min value in case the number of
   *         variables is higher than the number of
   *         constraints specified as strings.
   * @param def_max default max value in case the number of
   *         variables is higher than the number of
   *         constraints specified as strings.
   */
  constraints_str(const std::vector<std::string>& str, size_t var_count,
              double def_min, double def_max)
      : constraints(var_count, def_min, def_max) {
    for (std::vector<std::string>::size_type i = 0; i < str.size(); ++i) {
      tokenizer tokens(str[i], separator(";,"));

      std::string type;
      double _min;
      double _max;

      size_t count(0);

      for (tokenizer::const_iterator j = tokens.begin(); j != tokens.end();
           ++j, ++count) {
        const std::string token(boost::trim_copy(*j));

        try {
          switch (count) {
            case 0:
              type = token;
              break;
            case 1:
              _min = boost::lexical_cast<double>(token.c_str());
              break;
            case 2:
              _max = boost::lexical_cast<double>(token.c_str());
              break;
            default:
              // too many fields
              throw constraints_exception(
                  (boost::format(
                       "wrong variable format in \"%1%\" - too many fields") %
                   str[i])
                      .str());
          }
        } catch (const boost::bad_lexical_cast&) {
          throw constraints_exception(
              (boost::format("wrong floating point number format: %1%") % token)
                  .str());
        }
      }

      // too few fields
      if (count < 3)
        throw constraints_exception(
            (boost::format(
                 "wrong variable format in \"%1%\" - too few fields") %
             str[i])
                .str());

      if (i < var_count)
        constraints::at(i) = str_to_constraint(type, _min, _max);
      else
        constraints::push_back(str_to_constraint(type, _min, _max));
    }
  }

 private:
  constraint_ptr str_to_constraint(const std::string& type, double min,
                                   double max) {
    if (boost::to_lower_copy(type) == "real")
      return std::make_shared<real_constraint>(min, max);
    else if (boost::to_lower_copy(type) == "int" ||
             boost::to_lower_copy(type) == "integer")
      return std::make_shared<int_constraint>(min, max);
    else
      throw constraints_exception(
          (boost::format("invalid constraint type \"%1%\"") % type).str());
  }
};

} // namespace de
} // namespace amichel

#endif  // DE_TEST_CMDLINE_H_INCLUDED
