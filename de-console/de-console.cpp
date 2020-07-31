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

#include <test_listener.h>
#include <test_processor_listener.h>

#include "cmdline.h"

using namespace amichel::de;

/**
 * Runs the Differential Evolution optimization process on
 * function and with the parameters selected on the command line
 *
 * @param cmdLine
 */
void testFunctions(const CmdLine& cmdLine) {
  // get the constraints as defined on the command line
  constraints constraints(cmdLine.constraints());
  assert(constraints);

  // get the objective function as selected on the command line
  ObjectiveFunction of(cmdLine.functionToOptimize());
  assert(of);

  // instantiate a DE listener
  listener_ptr listener(std::make_shared<DETestListener>());

  // instantiate a Processors listener
  processor_listener_ptr processorListener(
      std::make_shared<DETestProcessorListener>());

  // instantiate the Processors, using the number of processors defined on the
  // command line, and the processors listener
  processors processors{ cmdLine.processorsCount(), of, processorListener };

  // instantiate a basic termination strategy (just count the # of generations)
  termination_strategy terminationStrategy(max_gen_termination_strategy(cmdLine.maxGenerations()));

  // instantiate the selection and mutation strategies as selected on the
  // command line
  selection_strategy selectionStrategy(cmdLine.selectionStrategy());
  mutation_strategy_ptr mutationStrategy(cmdLine.mutationStrategy());

  // show a message with some basic facts about the session
  std::cout << (cmdLine.minimize() ? "minimizing \"" : "maximizing \"")
            << /*of->name() <<*/ "\" with weight factor " << cmdLine.weight()
            << " and crossover factor " << cmdLine.crossover() << std::endl
            << std::endl;
  ;

  // create a differential_evolution object using all the parameters defined
  // above or on the command line
  differential_evolution de(
      cmdLine.argumentsCount(), cmdLine.populationSize(), processors,
      constraints, cmdLine.minimize(), terminationStrategy, selectionStrategy,
      mutationStrategy, listener);

  // run the optimization process
  de.run();
}

int main(int argc, char* argv[]) {
  try {
    // instantiate a command line object
    CmdLine cmdLine;

    // if command line processing was successful, run the test
    if (cmdLine.process(argc, argv)) testFunctions(cmdLine);

    return 0;
  }
  catch (const CmdLineException& e) {
    // there has been a cmd line error
    std::cout << "Command line parameter error: " << e.what() << std::endl;
    return 1;
  }
  catch (const exception& e) {
    // there has been some error, must likely triggered by a boost object.
    std::cout << e.what() << std::endl;
    return 1;
  }
  catch (const std::exception& e) {
    // catching all other exceptions, so the process won't crash
    // this most likely indicates a bug (memory issue, null pointer etc).
    std::cout << e.what() << std::endl;
  }
}
