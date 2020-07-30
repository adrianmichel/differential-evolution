/*
 * Copyright (c) 2017, Adrian Michel
 * http://www.amichel.com
 *
 * This software is released under the 3-Clause BSD License
 *
 * The complete terms can be found in the attached LICENSE file
 * or at https://opensource.org/licenses/BSD-3-Clause
 */

#include "cmdline.h"
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iostream>
#include "test_functions.h"

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/environment_iterator.hpp>
#include <boost/program_options/eof_iterator.hpp>
#include <boost/program_options/errors.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/version.hpp>
#include <boost/regex.hpp>

// useful types
using StrVector = std::vector<std::string>;
using StrVectorPtr = std::shared_ptr<StrVector>;
using Separator = boost::char_separator<char>;
using Tokenizer = boost::tokenizer<Separator>;

// helper macros
#define ARGS(x) x##_LONG "," x##_SHORT
#define SET(name, var, type)         \
  assert(vm.count(name##_LONG) > 0); \
  var = vm[name##_LONG].as<type>();
#define MAKE_MUTATION_STRATEGY(n)            \
  std::make_shared<mutation_strategy_##n>( \
      argumentsCount(), mutation_strategy_arguments(weight(), crossover()))

// namespaces used in this file
namespace po = boost::program_options;
using namespace std;
using namespace amichel::de;

// various constants
constexpr auto minWeight = 0;
constexpr auto maxWeight = 2;

constexpr auto minCrossover = 0;
constexpr auto maxCrossover = 1;

// single character options used
//
// abcdefghijklmnopqrstuvwxyz
//   ** ** *   **** *   ***
//
// ABCDEFGHIJKLMNOPQRSTUVWXYZ
//

// supported command line arguments definition
#define COMMAND_LINE_HELP_SHORT "?"
#define COMMAND_LINE_HELP_LONG "help"

#define WEIGHT_SHORT "w"  // weight factor
#define WEIGHT_LONG "weight"

#define CROSSOVER_SHORT "c"  // crossover factor
#define CROSSOVER_LONG "crossover"

#define POPULATION_SIZE_SHORT "p"
#define POPULATION_SIZE_LONG "populationsize"  // population size

#define MAX_GENERATIONS_SHORT "g"  // max # of generations
#define MAX_GENERATIONS_LONG "generations"

#define MUTATION_STRATEGY_SHORT "m"  // 1-5
#define MUTATION_STRATEGY_LONG "mutationstrategy"

#define SELECTION_STRATEGY_SHORT "s"  // 1-5
#define SELECTION_STRATEGY_LONG "selectionstrategy"

#define CONFIG_FILE_SHORT "f"
#define CONFIG_FILE_LONG "configfile"

#define MINIMIZE_SHORT "i"
#define MINIMIZE_LONG "minimize"

#define PROCESSORS_COUNT_SHORT "r"
#define PROCESSORS_COUNT_LONG "processorcount"

#define FUNCTION_TO_OPTIMIZE_SHORT "o"
#define FUNCTION_TO_OPTIMIZE_LONG "functiontooptimize"

#define CONSTRAINTS_SHORT "n"
#define CONSTRAINTS_LONG "constraint"

#define VARIABLES_COUNT_SHORT "v"
#define VARIABLES_COUNT_LONG "variablescount"

#define DEF_CONSTRAINT_MIN_SHORT "x"
#define DEF_CONSTRAINT_MIN_LONG "defconstraintmin"

#define DEF_CONSTRAINT_MAX_SHORT "y"
#define DEF_CONSTRAINT_MAX_LONG "defconstraintmax"

#define HELP_SHORT "?"
#define HELP_LONG "help"

CmdLine::CmdLine()
    : _weight(0),
      _crossover(0),
      _populationSize(0),
      _maxGenerations(0),
      _minimize(true),
      _processorsCount(0) {
  notice(std::cout);
}

std::string CmdLine::getUsage() const {
  std::ostringstream o;
  o << "Command line usage: " << std::endl
    << (std::string&)_description << std::endl;
  return o.str();
}

void CmdLine::showUsage() { std::cout << getUsage(); }

/**
 * Process the command line
 *
 * @param argc
 * @param argv
 *
 * @return bool
 */
bool CmdLine::process(int argc, char* argv[]) {
  try {
    po::options_description desc;
    desc.add_options()(ARGS(COMMAND_LINE_HELP), "shows this message")(
        ARGS(WEIGHT), po::value<double>()->required(),
        "differential evolution weight factor in the range [0,2]")(
        ARGS(CROSSOVER), po::value<double>()->required(),
        "differential evolution crossover factor in the range [0,1]")(
        ARGS(POPULATION_SIZE), po::value<size_t>()->required(),
        "population size")(ARGS(MAX_GENERATIONS),
                           po::value<size_t>()->required(), "max generations")(
        ARGS(MUTATION_STRATEGY), po::value<size_t>()->default_value(1),
        "mutation strategy - an integer between 1-5 to specify one of the 5 "
        "mutation strategies available")(
        ARGS(SELECTION_STRATEGY), po::value<size_t>()->default_value(1),
        "selection strategy:\n- 1: \"best parent child selection\"\n- 2: "
        "\"tournament selection\"")(ARGS(MINIMIZE),
                                    po::value<bool>()->required(),
                                    "0 to maximize, 1 to minimize")(
        ARGS(CONFIG_FILE), po::value<std::vector<std::string> >(),
        "configuration file")(ARGS(PROCESSORS_COUNT),
                              po::value<size_t>()->required(),
                              "number of processors to run in parallel")(
        ARGS(CONSTRAINTS), po::value<std::vector<std::string> >()->required(),
        "constraints - there can be any number of constraints, one per "
        "variable. Format: \"type;min;max\", where min and max are floating "
        "point values, and type can be:\n"
        "- real\n"
        "- int\n"
        "- set\n"
        "- boolean")(
        ARGS(VARIABLES_COUNT), po::value<size_t>()->default_value(20),
        "number of variables per individual - it is usually much higher than "
        "the number of variables required by the objecive function")(
        ARGS(DEF_CONSTRAINT_MIN), po::value<double>()->default_value(-1e6),
        "default min constraint - used for constraints that are not set on the "
        "command line")(ARGS(DEF_CONSTRAINT_MAX),
                        po::value<double>()->default_value(1e6),
                        "default max constraint - used for constraints that "
                        "are not set on the command line")(
        ARGS(FUNCTION_TO_OPTIMIZE), po::value<size_t>()->required(),
        "function to optimize:\n"
        "- 1: -(x^2) [max: 0]\n"
        "- 2: x^2 [min: 0]\n"
        "- 3: a simple function [min:-25]\n"
        "- 4: sphere function [min: 0]\n"
        "- 5: auckley function [min: 0]\n"
        "- 6: second De Jong function [min: 0]\n"
        "- 7: six hump camel back function [min: 1.031628453...]")

        ;

    std::ostringstream o;
    o << desc;

    _description = o.str();

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);

    if (vm.count(HELP_LONG)) {
      showUsage();
      return false;
    } else {
      // now loading the arguments from the configuration file(s) - there can be
      // any number of them
      if (vm.count(CONFIG_FILE_LONG) > 0) {
        std::vector<std::string> configFiles(
            vm[CONFIG_FILE_LONG].as<std::vector<std::string> >());

        for (auto configFile : configFiles) {
          std::ifstream ifs(configFile.c_str());
          if (ifs)
            po::store(po::parse_config_file(ifs, desc), vm);
          else
            throw CmdLineException(
                (boost::format("Couldn't open configuration file: \"%1%\"") %
                 configFile)
                    .str());
        }
      }

      po::notify(vm);

      double argumentsDefConstraintMin;
      double argumentsDefConstraintMax;

      size_t mutationStrategyIndex;
      size_t selectionStrategyIndex;

      SET(MINIMIZE, _minimize, bool)
      SET(WEIGHT, _weight, double)
      SET(CROSSOVER, _crossover, double)
      SET(POPULATION_SIZE, _populationSize, size_t)
      SET(MAX_GENERATIONS, _maxGenerations, size_t)
      SET(MUTATION_STRATEGY, mutationStrategyIndex, size_t)
      SET(PROCESSORS_COUNT, _processorsCount, size_t)
      SET(VARIABLES_COUNT, _argumentsCount, size_t)
      SET(DEF_CONSTRAINT_MIN, argumentsDefConstraintMin, double)
      SET(DEF_CONSTRAINT_MAX, argumentsDefConstraintMax, double)
      SET(SELECTION_STRATEGY, selectionStrategyIndex, size_t)

      if (weight() < minWeight || weight() > maxWeight)
        throw CmdLineException(
            (boost::format(
                 "Weight %1% out of range - it must be between [%2%, %3%]") %
             weight() % minWeight % maxWeight)
                .str());

      if (crossover() < minCrossover || crossover() > maxCrossover)
        throw CmdLineException(
            (boost::format(
                 "Crossover %1% out of range - it must be between [%2%, %3%]") %
             crossover() % minCrossover % maxCrossover)
                .str());

      assert(vm.count(CONSTRAINTS_LONG) > 0);
      _constraints = std::make_shared<amichel::de::constraints>(
          vm[CONSTRAINTS_LONG].as<std::vector<std::string> >(), _argumentsCount,
          argumentsDefConstraintMin, argumentsDefConstraintMax);

      assert(vm.count(FUNCTION_TO_OPTIMIZE_LONG) > 0);
      m_functionToOptimize = vm[FUNCTION_TO_OPTIMIZE_LONG].as<size_t>() - 1;

      switch (selectionStrategyIndex) {
        case 1:
          _selectionStrategy = best_parent_child_selection_strategy();
          break;
        case 2:
          _selectionStrategy = tournament_selection_strategy();
          break;
        default:
          throw CmdLineException(
              (boost::format("Invalid selection strategy: %1%") %
                selectionStrategyIndex)
                  .str());
      }

      switch (mutationStrategyIndex) {
        case 1:
          _mutationStrategy = MAKE_MUTATION_STRATEGY(1);
          break;
        case 2:
          _mutationStrategy = MAKE_MUTATION_STRATEGY(2);
          break;
        case 3:
          _mutationStrategy = MAKE_MUTATION_STRATEGY(3);
          break;
        case 4:
          _mutationStrategy = MAKE_MUTATION_STRATEGY(4);
          break;
        case 5:
          _mutationStrategy = MAKE_MUTATION_STRATEGY(5);
          break;
        default:
          throw CmdLineException(
              (boost::format("Invalid mutation strategy: %1%") %
               _mutationStrategy)
                  .str());
      }

      return true;
    }
  } catch (const constraints_exception& e) {
    throw CmdLineException(e.what());
  } catch (const CmdLineException&) {
    throw;
  } catch (const amichel::de::exception& e) {
    std::string newMessage(e.what());

    newMessage += (boost::format("\n\nSupported command line arguments:\n%1%") %
                   _description)
                      .str();

    throw CmdLineException(newMessage);
  }
}

void CmdLine::notice(std::ostream& os) {
  os << "-----------------------------------------------------------------------" << std::endl
     << "|   Differential Optimization (DE) C++ library and test application   |" << std::endl
     << "|                     Written by Adrian Michel                        |" << std::endl
     << "|                      http://www.amichel.com                         |" << std::endl
     << "|     Inspired by the C++ DE implementation by Dr. Rainer Storn at    |" << std::endl
     << "|           http://www.icsi.berkeley.edu/~storn/code.html             |" << std::endl
     << "-----------------------------------------------------------------------" << std::endl
     << std::endl;
}

ObjectiveFunction CmdLine::_functions[] = {
    x_sqr_max_function,
    x_sqr_min_function,
    AnotherSimpleFunction,
    SphereFunction,
    AckleyFunction,
    SecondDeJongFunction,
    SixHumpCamelBackFunction
};
